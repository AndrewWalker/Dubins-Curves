/*
 * Copyright (c) 2008-2018, Andrew Walker
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#ifdef WIN32
#define _USE_MATH_DEFINES
#endif
#include <math.h>
#include "DubinsCurve/DubinsCurve.h"

#define EPSILON (10e-10)

typedef enum
{
    L_SEG = 0,
    S_SEG = 1,
    R_SEG = 2
} SegmentType;

/* The segment types for each of the Path types */
const SegmentType DIRDATA[][3] = {
    {L_SEG, S_SEG, L_SEG},
    {L_SEG, S_SEG, R_SEG},
    {R_SEG, S_SEG, L_SEG},
    {R_SEG, S_SEG, R_SEG},
    {R_SEG, L_SEG, R_SEG},
    {L_SEG, R_SEG, L_SEG}};

typedef struct
{
    double alpha;
    double beta;
    double d;
    double sa;
    double sb;
    double ca;
    double cb;
    double c_ab;
    double d_sq;
} DubinsIntermediateResults;

int dubinsPathWord(DubinsIntermediateResults *in, DubinsPathType pathType, double out[3]);
int dubinsPathIntermediateResults(DubinsIntermediateResults *in, double initialPose[3], double finalPose[3], double rho);

/**
 * Floating point modulus suitable for rings
 *
 * fmod doesn't behave correctly for angular quantities, this function does
 */
double fmodr(double x, double y)
{
    return x - y * floor(x / y);
}

double mod2pi(double theta)
{
    return fmodr(theta, 2 * M_PI);
}

int dubinsPathShortestPath(DubinsPath *path, double initialPose[3], double finalPose[3], double rho)
{
    int countSegment, errorCode;
    DubinsIntermediateResults in;
    double params[3];
    double cost;
    double bestCost = INFINITY;
    int bestWord = -1;

    errorCode = dubinsPathIntermediateResults(&in, initialPose, finalPose, rho);

    if (errorCode != EDUBOK)
    {
        return errorCode;
    }
    else
    {
    }

    path->initialPose[0] = initialPose[0];
    path->initialPose[1] = initialPose[1];
    path->initialPose[2] = initialPose[2];
    path->rho = rho;

    for (countSegment = 0; countSegment < 6; countSegment++)
    {
        DubinsPathType pathType = (DubinsPathType)countSegment;
        errorCode = dubinsPathWord(&in, pathType, params);
        if (errorCode == EDUBOK)
        {
            cost = params[0] + params[1] + params[2];
            if (cost < bestCost)
            {
                bestWord = countSegment;
                bestCost = cost;
                path->param[0] = params[0];
                path->param[1] = params[1];
                path->param[2] = params[2];
                path->type = pathType;
            }
        }
    }
    if (bestWord == -1)
    {
        return EDUBNOPATH;
    }
    return EDUBOK;
}

int dubinsPath(DubinsPath *path, double q0[3], double q1[3], double rho, DubinsPathType pathType)
{
    int errorCode;
    DubinsIntermediateResults in;

    errorCode = dubinsPathIntermediateResults(&in, q0, q1, rho);
    if (errorCode == EDUBOK)
    {
        double params[3];
        errorCode = dubinsPathWord(&in, pathType, params);
        if (errorCode == EDUBOK)
        {
            path->param[0] = params[0];
            path->param[1] = params[1];
            path->param[2] = params[2];
            path->initialPose[0] = q0[0];
            path->initialPose[1] = q0[1];
            path->initialPose[2] = q0[2];
            path->rho = rho;
            path->type = pathType;
        }
    }
    return errorCode;
}

double dubinsPathLength(DubinsPath *path)
{
    double length = 0.;
    length += path->param[0];
    length += path->param[1];
    length += path->param[2];
    length = length * path->rho;
    return length;
}

double dubinsPathSegmentLength(DubinsPath *path, int i)
{
    if ((i < 0) || (i > 2))
    {
        return INFINITY;
    }
    return path->param[i] * path->rho;
}

double dubinsPathSegmentLengthNormalized(DubinsPath *path, int i)
{
    if ((i < 0) || (i > 2))
    {
        return INFINITY;
    }
    return path->param[i];
}

DubinsPathType dubinsPathType(DubinsPath *path)
{
    return path->type;
}

void dubinsPathSegment(double t, double qi[3], double qt[3], SegmentType type)
{
    double st = sin(qi[2]);
    double ct = cos(qi[2]);

    switch (type)
    {
    case L_SEG:
        qt[0] = +sin(qi[2] + t) - st;
        qt[1] = -cos(qi[2] + t) + ct;
        qt[2] = t;
        break;
    case R_SEG:
        qt[0] = -sin(qi[2] - t) + st;
        qt[1] = +cos(qi[2] - t) - ct;
        qt[2] = -t;
        break;
    case S_SEG:
        qt[0] = ct * t;
        qt[1] = st * t;
        qt[2] = 0.0;
        break;
    default:
        break;
    }

    qt[0] += qi[0];
    qt[1] += qi[1];
    qt[2] += qi[2];
}

int dubinsPathSample(DubinsPath *path, double t, double q[3])
{
    /* tprime is the normalised variant of the parameter t */
    double tprime = t / path->rho;
    double qi[3]; /* The translated initial configuration */
    double q1[3]; /* end-of segment 1 */
    double q2[3]; /* end-of segment 2 */
    const SegmentType *types = DIRDATA[path->type];
    double p1, p2;

    if (t < 0 || t > dubinsPathLength(path))
    {
        return EDUBPARAM;
    }
    else {}

    /* initial configuration */
    qi[0] = 0.0;
    qi[1] = 0.0;
    qi[2] = path->initialPose[2];

    /* generate the target configuration */
    p1 = path->param[0];
    p2 = path->param[1];

    dubinsPathSegment(p1, qi, q1, types[0]);
    dubinsPathSegment(p2, q1, q2, types[1]);

    if (tprime < p1)
    {
        dubinsPathSegment(tprime, qi, q, types[0]);
    }
    else if (tprime < (p1 + p2))
    {
        dubinsPathSegment(tprime - p1, q1, q, types[1]);
    }
    else
    {
        dubinsPathSegment(tprime - p1 - p2, q2, q, types[2]);
    }

    /* scale the target configuration, translate back to the original starting point */
    q[0] = q[0] * path->rho + path->initialPose[0];
    q[1] = q[1] * path->rho + path->initialPose[1];
    q[2] = mod2pi(q[2]);

    return EDUBOK;
}

int dubinsPathSampleMany(DubinsPath *path, double stepSize,
                         DubinsPathSamplingCallback cb, void *user_data)
{
    int retcode;
    double q[3];
    double x = 0.0;
    double length = dubinsPathLength(path);
    while (x < length)
    {
        dubinsPathSample(path, x, q);
        retcode = cb(q, x, user_data);
        if (retcode != 0)
        {
            return retcode;
        }
        x += stepSize;
    }
    return 0;
}

int dubinsPathEndpoint(DubinsPath *path, double q[3])
{
    return dubinsPathSample(path, dubinsPathLength(path) - EPSILON, q);
}

int dubinsPathExtractSubpath(DubinsPath *path, double t, DubinsPath *newpath)
{
    /* calculate the true parameter */
    double tprime = t / path->rho;

    if ((t < 0) || (t > dubinsPathLength(path)))
    {
        return EDUBPARAM;
    }

    /* copy most of the data */
    newpath->initialPose[0] = path->initialPose[0];
    newpath->initialPose[1] = path->initialPose[1];
    newpath->initialPose[2] = path->initialPose[2];
    newpath->rho = path->rho;
    newpath->type = path->type;

    /* fix the parameters */
    newpath->param[0] = fmin(path->param[0], tprime);
    newpath->param[1] = fmin(path->param[1], tprime - newpath->param[0]);
    newpath->param[2] = fmin(path->param[2], tprime - newpath->param[0] - newpath->param[1]);
    return 0;
}

int dubinsPathIntermediateResults(DubinsIntermediateResults *in, double q0[3], double q1[3], double rho)
{
    double dx, dy, D, d, theta, alpha, beta;
    if (rho <= 0.0)
    {
        return EDUBBADRHO;
    }

    dx = q1[0] - q0[0];
    dy = q1[1] - q0[1];
    D = sqrt(dx * dx + dy * dy);
    d = D / rho;
    theta = 0;

    /* test required to prevent domain errors if dx=0 and dy=0 */
    if (d > 0)
    {
        theta = mod2pi(atan2(dy, dx));
    }
    else {}
    
    alpha = mod2pi(q0[2] - theta);
    beta = mod2pi(q1[2] - theta);

    in->alpha = alpha;
    in->beta = beta;
    in->d = d;
    in->sa = sin(alpha);
    in->sb = sin(beta);
    in->ca = cos(alpha);
    in->cb = cos(beta);
    in->c_ab = cos(alpha - beta);
    in->d_sq = d * d;

    return EDUBOK;
}

int dubinsLSL(DubinsIntermediateResults *in, double out[3])
{
    double tmp0, tmp1, p_sq;

    tmp0 = in->d + in->sa - in->sb;
    p_sq = 2 + in->d_sq - (2 * in->c_ab) + (2 * in->d * (in->sa - in->sb));

    if (p_sq >= 0)
    {
        tmp1 = atan2((in->cb - in->ca), tmp0);
        out[0] = mod2pi(tmp1 - in->alpha);
        out[1] = sqrt(p_sq);
        out[2] = mod2pi(in->beta - tmp1);
        return EDUBOK;
    }
    return EDUBNOPATH;
}

int dubinsRSR(DubinsIntermediateResults *in, double out[3])
{
    double tmp0 = in->d - in->sa + in->sb;
    double p_sq = 2 + in->d_sq - (2 * in->c_ab) + (2 * in->d * (in->sb - in->sa));
    if (p_sq >= 0)
    {
        double tmp1 = atan2((in->ca - in->cb), tmp0);
        out[0] = mod2pi(in->alpha - tmp1);
        out[1] = sqrt(p_sq);
        out[2] = mod2pi(tmp1 - in->beta);
        return EDUBOK;
    }
    return EDUBNOPATH;
}

int dubinsLSR(DubinsIntermediateResults *in, double out[3])
{
    double p_sq = -2 + (in->d_sq) + (2 * in->c_ab) + (2 * in->d * (in->sa + in->sb));
    if (p_sq >= 0)
    {
        double p = sqrt(p_sq);
        double tmp0 = atan2((-in->ca - in->cb), (in->d + in->sa + in->sb)) - atan2(-2.0, p);
        out[0] = mod2pi(tmp0 - in->alpha);
        out[1] = p;
        out[2] = mod2pi(tmp0 - mod2pi(in->beta));
        return EDUBOK;
    }
    return EDUBNOPATH;
}

int dubinsRSL(DubinsIntermediateResults *in, double out[3])
{
    double p_sq = -2 + in->d_sq + (2 * in->c_ab) - (2 * in->d * (in->sa + in->sb));
    if (p_sq >= 0)
    {
        double p = sqrt(p_sq);
        double tmp0 = atan2((in->ca + in->cb), (in->d - in->sa - in->sb)) - atan2(2.0, p);
        out[0] = mod2pi(in->alpha - tmp0);
        out[1] = p;
        out[2] = mod2pi(in->beta - tmp0);
        return EDUBOK;
    }
    return EDUBNOPATH;
}

int dubinsRLR(DubinsIntermediateResults *in, double out[3])
{
    double tmp0 = (6. - in->d_sq + 2 * in->c_ab + 2 * in->d * (in->sa - in->sb)) / 8.;
    double phi = atan2(in->ca - in->cb, in->d - in->sa + in->sb);
    if (fabs(tmp0) <= 1)
    {
        double p = mod2pi((2 * M_PI) - acos(tmp0));
        double t = mod2pi(in->alpha - phi + mod2pi(p / 2.));
        out[0] = t;
        out[1] = p;
        out[2] = mod2pi(in->alpha - in->beta - t + mod2pi(p));
        return EDUBOK;
    }
    return EDUBNOPATH;
}

int dubinsLRL(DubinsIntermediateResults *in, double out[3])
{
    double tmp0 = (6. - in->d_sq + 2 * in->c_ab + 2 * in->d * (in->sb - in->sa)) / 8.;
    double phi = atan2(in->ca - in->cb, in->d + in->sa - in->sb);
    if (fabs(tmp0) <= 1)
    {
        double p = mod2pi(2 * M_PI - acos(tmp0));
        double t = mod2pi(-in->alpha - phi + p / 2.);
        out[0] = t;
        out[1] = p;
        out[2] = mod2pi(mod2pi(in->beta) - in->alpha - t + mod2pi(p));
        return EDUBOK;
    }
    return EDUBNOPATH;
}

int dubinsPathWord(DubinsIntermediateResults *in, DubinsPathType pathType, double out[3])
{
    int result;
    switch (pathType)
    {
    case LSL:
        result = dubinsLSL(in, out);
        break;
    case RSL:
        result = dubinsRSL(in, out);
        break;
    case LSR:
        result = dubinsLSR(in, out);
        break;
    case RSR:
        result = dubinsRSR(in, out);
        break;
    case LRL:
        result = dubinsLRL(in, out);
        break;
    case RLR:
        result = dubinsRLR(in, out);
        break;
    default:
        result = EDUBNOPATH;
    }
    return result;
}
