using System.Collections.Generic;
using GenericCode;

namespace DelaunatorSharp
{
    // Ported and based on https://github.com/kninnug/Constrainautor
    public class Constrainautor
    {
        private List<int> vertMap;
        private BoolSet flips;
        private BoolSet consd;
        private int loopMax;

        public Constrainautor()
        {
            vertMap = new List<int>();
            flips = new BoolSet();
            consd = new BoolSet();
        }

        public void Create(Delaunator del, List<ConstraintEdge> edges)
        {
            int coordsCount = del.coords.Count;
            int numPoints = coordsCount / 2;
            int numEdges = del.triangles.Count;
            loopMax = coordsCount * 3;

            // Map every vertex id to the right-most edge that points to that vertex
            vertMap.Resize(numPoints);
            for (int i = 0; i < numPoints; i++)
            {
                vertMap[i] = -1;
            }

            // Keep track of edges flipped while constraining
            flips.Create(numEdges);
            // Keep track of constrained edges
            consd.Create(numEdges);

            for (int e = 0; e < numEdges; e++)
            {
                int v = del.triangles[e];
                if (vertMap[v] == -1)
                {
                    UpdateVert(e, del);
                }
            }

            ConstrainAll(edges, del);
        }

        public void ClearTemporaryLists()
        {
            vertMap.Clear();
            flips.Clear();
            consd.Clear();
        }

        public void ConstrainOne(int segP1, int segP2, Delaunator del)
        {
            int start = vertMap[segP1];
            int edg = start;

            int iLoop = 0;
            // Loop over edges touching segP1
            do
            {
                iLoop++;
                if (iLoop > loopMax)
                {
                    return;
                }
                if (edg == -1)
                {
                    // This is sometimes randomly happening
                    UnityEngine.Debug.Log($"ConstrainOne edg == -1 {iLoop} {loopMax}");
                    return;
                }

                int p4 = del.triangles[edg];
                int nxt = NextEdge(edg);

                // Already constrained in reverse order
                if (p4 == segP2)
                {
                    Protect(edg, del);
                    return;
                }

                int opp = PrevEdge(edg);
                int p3 = del.triangles[opp];

                // Already constrained
                if (p3 == segP2)
                {
                    Protect(nxt, del);
                    return;
                }

                // Edge opposite segP1 intersects constraint
                if (IntersectSegments(segP1, segP2, p3, p4, del))
                {
                    edg = opp;
                    break;
                }

                int adj = del.halfedges[nxt];
                edg = adj;
            } while (edg != -1 && edg != start);

            int conEdge = edg;
            int rescan = -1;
            iLoop = 0;

            while (edg != -1)
            {
                iLoop++;
                if (iLoop > loopMax)
                {
                    return;
                }

                int adj = del.halfedges[edg];
                int bot = PrevEdge(edg);
                int top = PrevEdge(adj);
                int rgt = NextEdge(adj);

                bool convex = IntersectSegments(
                    del.triangles[edg],
                    del.triangles[adj],
                    del.triangles[bot],
                    del.triangles[top],
                    del
                );

                if (!convex)
                {
                    if (rescan == -1)
                    {
                        rescan = edg;
                    }

                    if (del.triangles[top] == segP2)
                    {
                        if (edg == rescan)
                        {
                            UnityEngine.Debug.Log("Infinite loop: non-convex quadrilateral");
                            return;
                        }
                        edg = rescan;
                        rescan = -1;
                        continue;
                    }

                    if (IntersectSegments(segP1, segP2, del.triangles[top], del.triangles[adj], del))
                    {
                        edg = top;
                    }
                    else if (IntersectSegments(segP1, segP2, del.triangles[rgt], del.triangles[top], del))
                    {
                        edg = rgt;
                    }
                    else if (rescan == edg)
                    {
                        // UnityEngine.Debug.Log("Infinite loop: no further intersect after non-convex");
                        return;
                    }

                    continue;
                }

                FlipDiagonal(edg, del);

                if (IntersectSegments(segP1, segP2, del.triangles[bot], del.triangles[top], del))
                {
                    if (rescan == -1)
                    {
                        rescan = bot;
                    }
                    if (rescan == bot)
                    {
                        // UnityEngine.Debug.Log("Infinite loop: flipped diagonal still intersects");
                        return;
                    }
                }

                if (del.triangles[top] == segP2)
                {
                    conEdge = top;
                    edg = rescan;
                    rescan = -1;
                }
                else if (IntersectSegments(segP1, segP2, del.triangles[rgt], del.triangles[top], del))
                {
                    edg = rgt;
                }
            }

            int halfedgesCount = del.halfedges.Count;
            if (conEdge <= -1 || conEdge >= halfedgesCount)
            {
                return;
            }

            Protect(conEdge, del);

            Delaunify(false, del);
        }

        public void ConstrainAll(List<ConstraintEdge> edges, Delaunator del)
        {
            for (int i = 0; i < edges.Count; i++)
            {
                ConstrainOne(edges[i].p, edges[i].q, del);
            }
        }

        public void Delaunify(bool deep, Delaunator del)
        {
            int len = del.halfedges.Count;
            int flipped;
            int iLoop = 0;

            do
            {
                iLoop++;
                if (iLoop > loopMax)
                {
                    return;
                }

                flipped = 0;
                for (int edg = 0; edg < len; edg++)
                {
                    if (consd.Has(edg))
                    {
                        continue;
                    }

                    flips.Remove(edg);
                    int adj = del.halfedges[edg];
                    if (adj == -1)
                    {
                        continue;
                    }

                    flips.Remove(adj);
                    if (!IsDelaunay(edg, del))
                    {
                        FlipDiagonal(edg, del);
                        flipped++;
                    }
                }
            } while (deep && flipped > 0);
        }

        private int Protect(int edg, Delaunator del)
        {
            int adj = del.halfedges[edg];
            flips.Remove(edg);
            consd.Add(edg);

            if (adj != -1)
            {
                flips.Remove(adj);
                consd.Add(adj);
                return adj;
            }

            return -edg;
        }

        private bool MarkFlip(int edg, Delaunator del)
        {
            if (consd.Has(edg))
            {
                return false;
            }

            int adj = del.halfedges[edg];
            if (adj != -1)
            {
                flips.Add(edg);
                flips.Add(adj);
            }
            return true;
        }

        private void FlipDiagonal(int edg, Delaunator del)
        {
            int adj = del.halfedges[edg];
            int bot = PrevEdge(edg);
            int lft = NextEdge(edg);
            int top = PrevEdge(adj);
            int rgt = NextEdge(adj);
            int adjBot = del.halfedges[bot];
            int adjTop = del.halfedges[top];

            if (consd.Has(edg))
            {
                // UnityEngine.Debug.Log("Trying to flip a constrained edge");
                return;
            }

            // Move edg to top
            del.triangles[edg] = del.triangles[top];
            del.halfedges[edg] = adjTop;
            if (!flips.Set(edg, flips.Has(top)))
            {
                consd.Set(edg, consd.Has(top));
            }
            if (adjTop != -1)
            {
                del.halfedges[adjTop] = edg;
            }
            del.halfedges[bot] = top;

            // Move adj to bot
            del.triangles[adj] = del.triangles[bot];
            del.halfedges[adj] = adjBot;
            if (!flips.Set(adj, flips.Has(bot)))
            {
                consd.Set(adj, consd.Has(bot));
            }
            if (adjBot != -1)
            {
                del.halfedges[adjBot] = adj;
            }
            del.halfedges[top] = bot;

            MarkFlip(edg, del);
            MarkFlip(lft, del);
            MarkFlip(adj, del);
            MarkFlip(rgt, del);

            flips.Add(bot);
            consd.Remove(bot);
            flips.Add(top);
            consd.Remove(top);

            UpdateVert(edg, del);
            UpdateVert(lft, del);
            UpdateVert(adj, del);
            UpdateVert(rgt, del);
        }

        private bool IsDelaunay(int edg, Delaunator del)
        {
            int adj = del.halfedges[edg];

            if (adj == -1)
            {
                return true;
            }

            int p1 = del.triangles[PrevEdge(edg)];
            int p2 = del.triangles[edg];
            int p3 = del.triangles[NextEdge(edg)];
            int px = del.triangles[PrevEdge(adj)];

            return !InCircle(p1, p2, p3, px, del);
        }

        private int UpdateVert(int start, Delaunator del)
        {
            int v = del.triangles[start];
            int inc = PrevEdge(start);
            int adj = del.halfedges[inc];

            while (adj != -1 && adj != start)
            {
                inc = PrevEdge(adj);
                adj = del.halfedges[inc];
            }

            vertMap[v] = inc;
            return inc;
        }

        // Geometric predicates
        private bool IntersectSegments(int p1, int p2, int p3, int p4, Delaunator del)
        {
            if (p1 == p3 || p1 == p4 || p2 == p3 || p2 == p4)
            {
                return false;
            }

            return IntersectSegments(
                del.coords[p1 * 2], del.coords[p1 * 2 + 1],
                del.coords[p2 * 2], del.coords[p2 * 2 + 1],
                del.coords[p3 * 2], del.coords[p3 * 2 + 1],
                del.coords[p4 * 2], del.coords[p4 * 2 + 1]
            );
        }

        private bool InCircle(int p1, int p2, int p3, int px, Delaunator del)
        {
            return InCircle(
                del.coords[p1 * 2], del.coords[p1 * 2 + 1],
                del.coords[p2 * 2], del.coords[p2 * 2 + 1],
                del.coords[p3 * 2], del.coords[p3 * 2 + 1],
                del.coords[px * 2], del.coords[px * 2 + 1]
            ) < 0.0f;
        }

        private int NextEdge(int e)
        {
            return (e % 3 == 2) ? e - 2 : e + 1;
        }

        private int PrevEdge(int e)
        {
            return (e % 3 == 0) ? e + 2 : e - 1;
        }

        public float Orient2D(float ax, float ay, float bx, float by, float cx, float cy)
        {
            float acx = ax - cx;
            float bcx = bx - cx;
            float acy = ay - cy;
            float bcy = by - cy;
            return acx * bcy - acy * bcx;
        }

        public float InCircle(float ax, float ay, float bx, float by, float cx, float cy, float dx, float dy)
        {
            float adx = ax - dx;
            float ady = ay - dy;
            float bdx = bx - dx;
            float bdy = by - dy;
            float cdx = cx - dx;
            float cdy = cy - dy;

            float abdet = adx * bdy - bdx * ady;
            float bcdet = bdx * cdy - cdx * bdy;
            float cadet = cdx * ady - adx * cdy;
            float alift = adx * adx + ady * ady;
            float blift = bdx * bdx + bdy * bdy;
            float clift = cdx * cdx + cdy * cdy;

            return alift * bcdet + blift * cadet + clift * abdet;
        }

        public bool IntersectSegments(float p1x, float p1y, float p2x, float p2y, float p3x, float p3y, float p4x, float p4y)
        {
            float x0 = Orient2D(p1x, p1y, p3x, p3y, p4x, p4y);
            float y0 = Orient2D(p2x, p2y, p3x, p3y, p4x, p4y);

            if ((x0 > 0 && y0 > 0) || (x0 < 0 && y0 < 0))
            {
                return false;
            }

            float x1 = Orient2D(p3x, p3y, p1x, p1y, p2x, p2y);
            float y1 = Orient2D(p4x, p4y, p1x, p1y, p2x, p2y);

            if ((x1 > 0 && y1 > 0) || (x1 < 0 && y1 < 0))
                return false;

            float epsilon = 0.0001f;
            // Check for degenerate collinear case
            // if (x0 == 0 && y0 == 0 && x1 == 0 && y1 == 0)
            if (x0 < epsilon && x0 > -epsilon &&
                y0 < epsilon && y0 > -epsilon &&
                x1 < epsilon && x1 > -epsilon &&
                y1 < epsilon && y1 > -epsilon)
            {
                return !(MathUtils.Max(p3x, p4x) < MathUtils.Min(p1x, p2x) ||
                        MathUtils.Max(p1x, p2x) < MathUtils.Min(p3x, p4x) ||
                        MathUtils.Max(p3y, p4y) < MathUtils.Min(p1y, p2y) ||
                        MathUtils.Max(p1y, p2y) < MathUtils.Min(p3y, p4y));
            }

            return true;
        }
    }
}
