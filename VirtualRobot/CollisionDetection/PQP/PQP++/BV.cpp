/*************************************************************************\

  Copyright 1999 The University of North Carolina at Chapel Hill.
  All Rights Reserved.

  Permission to use, copy, modify and distribute this software and its
  documentation for educational, research and non-profit purposes, without
  fee, and without a written agreement is hereby granted, provided that the
  above copyright notice and the following three paragraphs appear in all
  copies.

  IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL BE
  LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
  CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE
  USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY
  OF NORTH CAROLINA HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGES.

  THE UNIVERSITY OF NORTH CAROLINA SPECIFICALLY DISCLAIM ANY
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE
  PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND THE UNIVERSITY OF
  NORTH CAROLINA HAS NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT,
  UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

  The authors may be contacted via:

  US Mail:             E. Larsen
                       Department of Computer Science
                       Sitterson Hall, CB #3175
                       University of N. Carolina
                       Chapel Hill, NC 27599-3175

  Phone:               (919)962-1749

  EMail:               geom@cs.unc.edu


\**************************************************************************/

#include <algorithm>

#include <stdlib.h>
#include <math.h>

#include "BV.h"
#include "MatVec.h"
#include "RectDist.h"
#include "OBB_Disjoint.h"


namespace PQP
{
    void
    BV::FitToTris(PQP_REAL O[3][3], Tri* tris, int num_tris)
    {
        // store orientation

        pqp_math.McM(R, O);

        // project points of tris to R coordinates

        int num_points = 3 * num_tris;
        PQP_REAL(*P)[3] = new PQP_REAL[num_points][3];
        int point = 0;
        int i;

        for (i = 0; i < num_tris; i++)
        {
            pqp_math.MTxV(P[point], R, tris[i].p1);
            point++;

            pqp_math.MTxV(P[point], R, tris[i].p2);
            point++;

            pqp_math.MTxV(P[point], R, tris[i].p3);
            point++;
        }


#if PQP_BV_TYPE & OBB_TYPE
        {
            PQP_REAL minx, maxx, miny, maxy, minz, maxz, c[3];
            minx = maxx = P[0][0];
            miny = maxy = P[0][1];
            minz = maxz = P[0][2];

            for (i = 1; i < num_points; i++)
            {
                if (P[i][0] < minx)
                {
                    minx = P[i][0];
                }
                else if (P[i][0] > maxx)
                {
                    maxx = P[i][0];
                }

                if (P[i][1] < miny)
                {
                    miny = P[i][1];
                }
                else if (P[i][1] > maxy)
                {
                    maxy = P[i][1];
                }

                if (P[i][2] < minz)
                {
                    minz = P[i][2];
                }
                else if (P[i][2] > maxz)
                {
                    maxz = P[i][2];
                }
            }

            c[0] = (PQP_REAL)0.5 * (maxx + minx);
            c[1] = (PQP_REAL)0.5 * (maxy + miny);
            c[2] = (PQP_REAL)0.5 * (maxz + minz);
            pqp_math.MxV(To, R, c);

            d[0] = (PQP_REAL)0.5 * (maxx - minx);
            d[1] = (PQP_REAL)0.5 * (maxy - miny);
            d[2] = (PQP_REAL)0.5 * (maxz - minz);
        }
#endif

#if PQP_BV_TYPE & RSS_TYPE
        {
            PQP_REAL minx, maxx, miny, maxy, c[3];

            // compute thickness, which determines radius, and z of rectangle corner
            PQP_REAL cz, radsqr;
            {
                PQP_REAL minz = P[0][2];
                PQP_REAL maxz = P[0][2];

                for (i = 1; i < num_points; i++)
                {
                    if (P[i][2] < minz)
                    {
                        minz = P[i][2];
                    }
                    else if (P[i][2] > maxz)
                    {
                        maxz = P[i][2];
                    }
                }
                r = (PQP_REAL)0.5 * (maxz - minz);
                radsqr = r * r;
                cz = (PQP_REAL)0.5 * (maxz + minz);
            }

            // compute an initial length of rectangle along x direction
            // find minx and maxx as starting points

            {
                int x_minindex = 0;
                int x_maxindex = 0;
                for (i = 1; i < num_points; i++)
                {
                    if (P[i][0] < P[x_minindex][0])
                    {
                        x_minindex = i;
                    }
                    else if (P[i][0] > P[x_maxindex][0])
                    {
                        x_maxindex = i;
                    }
                }

                PQP_REAL dz;
                dz = P[x_minindex][2] - cz;
                minx = P[x_minindex][0] + sqrt(std::max(radsqr - dz * dz, 0.f));
                dz = P[x_maxindex][2] - cz;
                maxx = P[x_maxindex][0] - sqrt(std::max(radsqr - dz * dz, 0.f));
            }

            // grow minx / maxx
            {
                PQP_REAL x;
                for (i = 0; i < num_points; i++)
                {
                    // grow minx
                    if (P[i][0] < minx)
                    {
                        const PQP_REAL dz = P[i][2] - cz;
                        x = P[i][0] + sqrt(std::max(radsqr - dz * dz, 0.f));

                        if (x < minx)
                        {
                            minx = x;
                        }
                    }

                    // grow maxx
                    if (P[i][0] > maxx)
                    {
                        const PQP_REAL dz = P[i][2] - cz;
                        x = P[i][0] - sqrt(std::max(radsqr - dz * dz, 0.f));

                        if (x > maxx)
                        {
                            maxx = x;
                        }
                    }
                }
            }

            // compute an initial length of rectangle along y direction
            // find miny and maxy as starting points

            {
                int y_minindex = 0;
                int y_maxindex = 0;
                for (i = 1; i < num_points; i++)
                {
                    if (P[i][1] < P[y_minindex][1])
                    {
                        y_minindex = i;
                    }
                    else if (P[i][1] > P[y_maxindex][1])
                    {
                        y_maxindex = i;
                    }
                }

                PQP_REAL dz;
                dz = P[y_minindex][2] - cz;
                miny = P[y_minindex][1] + sqrt(std::max(radsqr - dz * dz, 0.f));
                dz = P[y_maxindex][2] - cz;
                maxy = P[y_maxindex][1] - sqrt(std::max(radsqr - dz * dz, 0.f));
            }

            // grow miny / maxy
            {
                PQP_REAL y;
                for (i = 0; i < num_points; i++)
                {
                    // grow miny
                    if (P[i][1] < miny)
                    {
                        const PQP_REAL dz = P[i][2] - cz;
                        y = P[i][1] + sqrt(std::max(radsqr - dz * dz, 0.f));

                        if (y < miny)
                        {
                            miny = y;
                        }
                    }

                    // grow maxy
                    if (P[i][1] > maxy)
                    {
                        const PQP_REAL dz = P[i][2] - cz;
                        y = P[i][1] - sqrt(std::max(radsqr - dz * dz, 0.f));

                        if (y > maxy)
                        {
                            maxy = y;
                        }
                    }
                }
            }

            // corners may have some points which are not covered - grow lengths if
            // necessary

            PQP_REAL dx, dy, u, t;
            PQP_REAL a = sqrt((PQP_REAL)0.5);

            for (i = 0; i < num_points; i++)
            {
                if (P[i][0] > maxx)
                {
                    if (P[i][1] > maxy)
                    {
                        dx = P[i][0] - maxx;
                        dy = P[i][1] - maxy;
                        u = dx * a + dy * a;
                        t = (a * u - dx) * (a * u - dx) +
                                (a * u - dy) * (a * u - dy) +
                                (cz - P[i][2]) * (cz - P[i][2]);
                        u = u - sqrt(std::max(radsqr - t, 0.f));

                        if (u > 0)
                        {
                            maxx += u * a;
                            maxy += u * a;
                        }
                    }
                    else if (P[i][1] < miny)
                    {
                        dx = P[i][0] - maxx;
                        dy = P[i][1] - miny;
                        u = dx * a - dy * a;
                        t = (a * u - dx) * (a * u - dx) +
                                (-a * u - dy) * (-a * u - dy) +
                                (cz - P[i][2]) * (cz - P[i][2]);
                        u = u - sqrt(std::max(radsqr - t, 0.f));

                        if (u > 0)
                        {
                            maxx += u * a;
                            miny -= u * a;
                        }
                    }
                }
                else if (P[i][0] < minx)
                {
                    if (P[i][1] > maxy)
                    {
                        dx = P[i][0] - minx;
                        dy = P[i][1] - maxy;
                        u = dy * a - dx * a;
                        t = (-a * u - dx) * (-a * u - dx) +
                                (a * u - dy) * (a * u - dy) +
                                (cz - P[i][2]) * (cz - P[i][2]);
                        u = u - sqrt(std::max(radsqr - t, 0.f));

                        if (u > 0)
                        {
                            minx -= u * a;
                            maxy += u * a;
                        }
                    }
                    else if (P[i][1] < miny)
                    {
                        dx = P[i][0] - minx;
                        dy = P[i][1] - miny;
                        u = -dx * a - dy * a;
                        t = (-a * u - dx) * (-a * u - dx) +
                                (-a * u - dy) * (-a * u - dy) +
                                (cz - P[i][2]) * (cz - P[i][2]);
                        u = u - sqrt(std::max(radsqr - t, 0.f));

                        if (u > 0)
                        {
                            minx -= u * a;
                            miny -= u * a;
                        }
                    }
                }
            }

            c[0] = minx;
            c[1] = miny;
            c[2] = cz;
            pqp_math.MxV(Tr, R, c);

            l[0] = maxx - minx;

            if (l[0] < 0)
            {
                l[0] = 0;
            }

            l[1] = maxy - miny;

            if (l[1] < 0)
            {
                l[1] = 0;
            }
        }

#endif

        delete [] P;
    }

    int
    BV_Processor::BV_Overlap(PQP_REAL R[3][3], PQP_REAL T[3], BV* b1, BV* b2)
    {
#if PQP_BV_TYPE & OBB_TYPE
        return (o.obb_disjoint(R, T, b1->d, b2->d) == 0);
#else
        PQP_REAL dist = p.RectDist(R, T, b1->l, b2->l);

        if (dist <= (b1->r + b2->r))
        {
            return 1;
        }

        return 0;
#endif
    }

#if PQP_BV_TYPE & RSS_TYPE
    PQP_REAL
    BV_Processor::BV_Distance(PQP_REAL R[3][3], PQP_REAL T[3], BV* b1, BV* b2)
    {
        PQP_REAL dist = p.RectDist(R, T, b1->l, b2->l);
        dist -= (b1->r + b2->r);
        return (dist < (PQP_REAL)0.0) ? (PQP_REAL)0.0 : dist;
    }
#endif

} // namespace

