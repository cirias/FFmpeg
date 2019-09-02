/*
 * Copyright 2007 Bobby Bingham
 * Copyright 2012 Robert Nagy <ronag89 gmail com>
 * Copyright 2012 Anton Khirnov <anton khirnov net>
 * Copyright 2018 Calvin Walton <calvin.walton@kepstin.ca>
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * @file
 * a filter enforcing given constant framerate
 */

#include <float.h>
#include <stdint.h>

#include "libavutil/avassert.h"
#include "libavutil/mathematics.h"
#include "libavutil/opt.h"
#include "avfilter.h"
#include "filters.h"
#include "internal.h"

enum EOFAction {
    EOF_ACTION_ROUND,
    EOF_ACTION_PASS,
    EOF_ACTION_NB
};

typedef struct FPSPreciselyContext {
    const AVClass *class;

    AVRational framerate;   ///< target framerate

    int64_t  timebase_den;     ///< shared denominator of timebase for both input and output
    int64_t  in_timebase_num;  ///< numerator of input timebase
    int64_t  out_timebase_num; ///< numerator of output timebase

    int64_t  in_start_pts;     ///< pts of the first frame of input
    int64_t  out_next_pts;     ///< pts of the next frame to output

    /* Runtime state */
    int      status;           ///< buffered input status

    AVFrame *frames[2];        ///< buffered frames
    int      frames_count;     ///< number of buffered frames

    // int64_t  next_pts;      

    /* statistics */
    int cur_frame_out;         ///< number of times current frame has been output
    int frames_in;             ///< number of frames on input
    int frames_out;            ///< number of frames on output
    int dup;                   ///< number of frames duplicated
    int drop;                  ///< number of framed dropped
} FPSPreciselyContext;

#define OFFSET(x) offsetof(FPSPreciselyContext, x)
#define V AV_OPT_FLAG_VIDEO_PARAM
#define F AV_OPT_FLAG_FILTERING_PARAM
static const AVOption fps_precisely_options[] = {
    { "fps", "A string describing desired output framerate", OFFSET(framerate), AV_OPT_TYPE_VIDEO_RATE, { .str = "25" }, 0, INT_MAX, V|F },
    { NULL }
};

AVFILTER_DEFINE_CLASS(fps_precisely);

static av_cold int init(AVFilterContext *ctx)
{
    FPSPreciselyContext *s = ctx->priv;

    s->in_start_pts = AV_NOPTS_VALUE;

    av_log(ctx, AV_LOG_VERBOSE, "fps=%d/%d\n", s->framerate.num, s->framerate.den);
    return 0;
}

/* Remove the first frame from the buffer, returning it */
static AVFrame *shift_frame(AVFilterContext *ctx, FPSPreciselyContext *s)
{
    AVFrame *frame;

    /* Must only be called when there are frames in the buffer */
    av_assert1(s->frames_count > 0);

    frame = s->frames[0];
    s->frames[0] = s->frames[1];
    s->frames[1] = NULL;
    s->frames_count--;

    /* Update statistics counters */
    s->frames_out += s->cur_frame_out;
    if (s->cur_frame_out > 1) {
        av_log(ctx, AV_LOG_DEBUG, "Duplicated frame with pts %"PRId64" %d times\n",
               frame->pts, s->cur_frame_out - 1);
        s->dup += s->cur_frame_out - 1;
    } else if (s->cur_frame_out == 0) {
        av_log(ctx, AV_LOG_DEBUG, "Dropping frame with pts %"PRId64"\n",
               frame->pts);
        s->drop++;
    }
    s->cur_frame_out = 0;

    return frame;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    FPSPreciselyContext *s = ctx->priv;

    AVFrame *frame;

    while (s->frames_count > 0) {
        frame = shift_frame(ctx, s);
        av_frame_free(&frame);
    }

    av_log(ctx, AV_LOG_VERBOSE, "%d frames in, %d frames out; %d frames dropped, "
           "%d frames duplicated.\n", s->frames_in, s->frames_out, s->drop, s->dup);
}

static int config_props(AVFilterLink* outlink)
{
    AVFilterContext *ctx    = outlink->src;
    FPSPreciselyContext      *s      = ctx->priv;

    outlink->time_base  = av_inv_q(s->framerate);
    outlink->frame_rate = s->framerate;

    return 0;
}

static int config_timebase(AVFilterContext *ctx, AVFilterLink *inlink, AVFilterLink *outlink)
{
    FPSPreciselyContext *s = ctx->priv;
    if (s->timebase_den != 0 && s->in_timebase_num != 0 && s->out_timebase_num != 0) {
        return 0;
    }

    int64_t gcd = av_gcd(inlink->time_base.den, outlink->time_base.den);
    int64_t lcm = (inlink->time_base.den / gcd) * outlink->time_base.den;

    s->timebase_den = lcm;
    s->in_timebase_num = inlink->time_base.num * (s->timebase_den / inlink->time_base.den);
    s->out_timebase_num = outlink->time_base.num * (s->timebase_den / outlink->time_base.den);

    av_log(ctx, AV_LOG_DEBUG,
        "Set timebase_den %"PRId64", in_timebase_num %"PRId64", out_timebase_num %"PRId64"\n",
        s->timebase_den, s->in_timebase_num, s->out_timebase_num);

    return 0;
}

/* Read a frame from the input and save it in the buffer */
static int read_frame(AVFilterContext *ctx, FPSPreciselyContext *s, AVFilterLink *inlink, AVFilterLink *outlink)
{
    AVFrame *frame;
    int ret;

    /* Must only be called when we have buffer room available */
    av_assert1(s->frames_count < 2);

    ret = ff_inlink_consume_frame(inlink, &frame);
    /* Caller must have run ff_inlink_check_available_frame first */
    av_assert1(ret);
    if (ret < 0)
        return ret;

    if (s->in_start_pts == AV_NOPTS_VALUE) {
        s->in_start_pts = frame->pts;
    }

    frame->pts = frame->pts - s->in_start_pts;

    s->frames[s->frames_count++] = frame;
    s->frames_in++;

    return 1;
}

/* Write a frame to the output */
static int write_frame(AVFilterContext *ctx, FPSPreciselyContext *s, AVFilterLink *outlink, int *again)
{
    AVFrame *frame;

    av_assert1(s->frames_count == 2 || (s->status && s->frames_count == 1));

    int64_t in_timestamps[2] = {};
    for (int i = 0; i < s->frames_count; i++) {
        in_timestamps[i] = s->frames[i]->pts * s->in_timebase_num;
        av_log(ctx, AV_LOG_VERBOSE, "in_timestamps[%d] %"PRId64"\n", i, in_timestamps[i]);
    }

    int64_t out_timestamp = s->out_next_pts * s->out_timebase_num;
    av_log(ctx, AV_LOG_VERBOSE, "out_timestamp %"PRId64"\n", out_timestamp);


    /* There are two conditions where we want to drop a frame:
     * - If we have two buffered frames and the second frame is more close
     *   to the next output frame, then drop the first buffered frame.
     * - If we have status (EOF) set, drop frames when we hit the
     *   status timestamp. */
    if ((s->frames_count == 2 && abs(in_timestamps[0] - out_timestamp) > abs(in_timestamps[1] - out_timestamp)) ||
        (s->frames_count == 1 && in_timestamps[0] < out_timestamp)) {

        frame = shift_frame(ctx, s);
        av_frame_free(&frame);
        *again = 1;
        return 0;
    }

    /* Output a copy of the first buffered frame */
    frame = av_frame_clone(s->frames[0]);
    if (!frame)
        return AVERROR(ENOMEM);
    // Make sure Closed Captions will not be duplicated
    av_frame_remove_side_data(s->frames[0], AV_FRAME_DATA_A53_CC);
    frame->pts = s->out_next_pts++;

    av_log(ctx, AV_LOG_DEBUG, "Writing frame with pts %"PRId64" to pts %"PRId64"\n",
           s->frames[0]->pts, frame->pts);
    s->cur_frame_out++;

    return ff_filter_frame(outlink, frame);
}

static int activate(AVFilterContext *ctx)
{
    FPSPreciselyContext   *s       = ctx->priv;
    AVFilterLink *inlink  = ctx->inputs[0];
    AVFilterLink *outlink = ctx->outputs[0];

    config_timebase(ctx, inlink, outlink);

    int ret;
    int again = 0;
    int64_t status_pts;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    /* No buffered status: normal operation */
    if (!s->status) {

        /* Read available input frames if we have room */
        while (s->frames_count < 2 && ff_inlink_check_available_frame(inlink)) {
            ret = read_frame(ctx, s, inlink, outlink);
            if (ret < 0)
                return ret;
        }

        /* We do not yet have enough frames to produce output */
        if (s->frames_count < 2) {
            /* Check if we've hit EOF (or otherwise that an error status is set) */
            ret = ff_inlink_acknowledge_status(inlink, &s->status, &status_pts);
            if (ret > 0) {
                av_log(ctx, AV_LOG_DEBUG, "EOF is at pts %"PRId64"\n", status_pts);
            }

            if (!ret) {
                /* If someone wants us to output, we'd better ask for more input */
                FF_FILTER_FORWARD_WANTED(outlink, inlink);
                return 0;
            }
        }
    }

    /* Buffered frames are available, so generate an output frame */
    if (s->frames_count > 0) {
        ret = write_frame(ctx, s, outlink, &again);
        /* Couldn't generate a frame, so schedule us to perform another step */
        if (again)
            ff_filter_set_ready(ctx, 100);
        return ret;
    }

    /* No frames left, so forward the status */
    if (s->status && s->frames_count == 0) {
        ff_outlink_set_status(outlink, s->status, s->out_next_pts);
        return 0;
    }

    return FFERROR_NOT_READY;
}

static const AVFilterPad avfilter_vf_fps_precisely_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
    },
    { NULL }
};

static const AVFilterPad avfilter_vf_fps_precisely_outputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_VIDEO,
        .config_props  = config_props,
    },
    { NULL }
};

AVFilter ff_vf_fps_precisely = {
    .name        = "fps_precisely",
    .description = NULL_IF_CONFIG_SMALL("Force constant framerate."),
    .init        = init,
    .uninit      = uninit,
    .priv_size   = sizeof(FPSPreciselyContext),
    .priv_class  = &fps_precisely_class,
    .activate    = activate,
    .inputs      = avfilter_vf_fps_precisely_inputs,
    .outputs     = avfilter_vf_fps_precisely_outputs,
};
