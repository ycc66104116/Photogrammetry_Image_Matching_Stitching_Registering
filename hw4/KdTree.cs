using System;
using System.Collections;

namespace NSURF
{
    public class SortedLimitedList : ArrayList
    {
        int max;

        public SortedLimitedList(int maxElements) : base(maxElements)
        {
            max = maxElements;
        }

        //	Processes list from right to left, sliding each node that is greater
        //	than 'this' to the right.  The loop ends when either the first node is
        //	reached, meaning obj is a new minimum, or it's proper sorted position
        //	in the list is reached.
        //	Returns position of obj or -1 if obj was not placed.

        public override int Add(object obj)
        {
            int pos = Count;

            while (pos > 0 && ((IComparable)base[pos - 1]).CompareTo(obj) >= 0)
            {
                if (pos < max)
                {
                    Set(pos, base[pos - 1]);
                }
                pos--;
            }

            if (pos < max)
            {
                Set(pos, obj);
            }
            else
            {
                pos = -1;
            }

            return pos;
        }

        // Sets the argument index to the argument object.
        // Replaces the node if it already exists,
        // adds a new node if at the end of the list,
        // does nothing otherwise.
        internal void Set(int idx, object obj)
        {
            if (idx < Count)
            {
                base[idx] = obj;
            }
            else if (idx == Count)
            {
                base.Add(obj);
            }
        }
    }


    public class KDTree
    {
        // The current element
        int splitDim;
        KDTree Left;
        KDTree Right;
        float[] KP;

        private KDTree()
        {
        }


        public struct BestEntry : IComparable
        {
            public float Distance;
            public float[] Neighbour;

            internal BestEntry(float[] Neighbour, float Distance)
            {
                this.Neighbour = Neighbour;
                this.Distance = Distance;
            }

            public int CompareTo(object obj)
            {
                BestEntry be = (BestEntry)obj;

                if (Distance < be.Distance)
                    return (-1);
                else if (Distance > be.Distance)
                    return (1);

                return (0);
            }
        }

        internal struct HREntry : IComparable
        {
            double Dist;
            public HyperRectangle Rect;
            public float[] Pivot;
            public KDTree Tree;


            internal HREntry(HyperRectangle Rect, KDTree Tree, float[] Pivot, double Dist)
            {
                this.Rect = Rect;
                this.Tree = Tree;
                this.Pivot = Pivot;
                this.Dist = Dist;

            }

            public int CompareTo(object obj)
            {
                HREntry hre = (HREntry)obj;

                if (Dist < hre.Dist)
                    return (-1);
                else if (Dist > hre.Dist)
                    return (1);

                return (0);
            }
        }

        internal struct HyperRectangle : ICloneable
        {
            float[] leftTop;
            float[] rightBottom;

            public object Clone()
            {
                HyperRectangle rec = new HyperRectangle
                {
                    leftTop = new float[64],
                    rightBottom = new float[64]
                };
                for (int n = 0; n < 64; ++n)
                {
                    rec.leftTop[n] = leftTop[n];
                    rec.rightBottom[n] = rightBottom[n];
                }

                return (rec);
            }

            static internal HyperRectangle CreateUniverseRectangle()
            {
                HyperRectangle rec = new HyperRectangle
                {
                    leftTop = new float[64],
                    rightBottom = new float[64]
                };

                for (int n = 0; n < 64; ++n)
                {
                    rec.leftTop[n] = -1;
                    rec.rightBottom[n] = 1;
                }

                return (rec);
            }

            internal HyperRectangle SplitAt(int splitDim, float splitVal)
            {
                if (leftTop[splitDim] >= splitVal || rightBottom[splitDim] < splitVal)
                    throw (new ArgumentException("SplitAt with splitpoint outside rec"));

                HyperRectangle r2 = (HyperRectangle)Clone();


                rightBottom[splitDim] = splitVal;
                r2.leftTop[splitDim] = splitVal;

                return (r2);
            }

            // Return true if any part of this HR is reachable from target by no
            // more than 'distRad', false otherwise.
            // The algorithm is specified in the kd-tree paper mentioned at the
            // top of this file, in section 6-7. But there is a mistake in the
            // third distinct case, which should read "hrMax" instead of "hrMin".
            internal bool IsInReach(float[] target, double distRad)
            {
                return (Distance(target) < distRad);
            }

            // Return the distance from the nearest point from within the HR to
            // the target point.
            internal double Distance(float[] target)
            {
                float closestPointN;
                float distance = 0;

                // first compute the closest point within hr to the target. if
                // this point is within reach of target, then there is an
                // intersection between the hypersphere around target with radius
                // 'dist' and this hyperrectangle.
                for (int n = 0; n < 64; ++n)
                {
                    float tI = target[n];
                    float hrMin = leftTop[n];
                    float hrMax = rightBottom[n];

                    closestPointN = 0;
                    if (tI <= hrMin)
                    {
                        closestPointN = hrMin;
                    }
                    else if (tI > hrMin && tI < hrMax)
                    {
                        closestPointN = tI;
                    }
                    else if (tI >= hrMax)
                    {
                        closestPointN = hrMax;
                    }

                    float dimDist = tI - closestPointN;
                    distance += dimDist * dimDist;
                }

                return (Math.Sqrt(distance));
            }
        }

        public float[] BestKP(float[] target, int searchSteps, float Thres)
        {
            float[] KPN = new float[2];
            HyperRectangle hr = HyperRectangle.CreateUniverseRectangle();

            SortedLimitedList best = new SortedLimitedList(2);
            SortedLimitedList searchHr = new SortedLimitedList(searchSteps);

            NearestNeighbourListBBFI(best, target, hr, 1, out float dummyDist, searchHr, ref searchSteps);
            BestEntry Best1 = (BestEntry)best[0];
            BestEntry Best2 = (BestEntry)best[1];

            if (Best1.Distance / Best2.Distance < Thres)
            {
                return Best1.Neighbour;
            }
            else
            {
                return null;
            }

        }


        private float[] NearestNeighbourListBBFI(SortedLimitedList best, float[] target, HyperRectangle hr, float MaxDist, out float resDistSq, SortedLimitedList searchHr, ref int searchSteps)
        {
            resDistSq = 1;

            float[] pivot = KP;

            best.Add(new BestEntry(pivot, GetDistance(target, pivot)));

            HyperRectangle leftHr = hr;
            HyperRectangle rightHr = leftHr.SplitAt(splitDim, pivot[splitDim]);
            HyperRectangle nearerHr, furtherHr;
            KDTree nearerKd, furtherKd;

            // step 5-7
            if (target[splitDim] <= pivot[splitDim])
            {
                nearerKd = Left;
                nearerHr = leftHr;
                furtherKd = Right;
                furtherHr = rightHr;
            }
            else
            {
                nearerKd = Right;
                nearerHr = rightHr;
                furtherKd = Left;
                furtherHr = leftHr;
            }

            // step 8
            float[] nearest = new float[66];
            float Distance;

            searchHr.Add(new HREntry(furtherHr, furtherKd, pivot, furtherHr.Distance(target)));

            // No child, bottom reached!
            if (nearerKd == null)
            {
                Distance = 1;
            }
            else
            {
                nearest = nearerKd.NearestNeighbourListBBFI(best, target, nearerHr, MaxDist, out Distance, searchHr, ref searchSteps);
            }

            // step 9
            if (best.Count >= 2)
            {
                MaxDist = ((BestEntry)best[1]).Distance;
            }
            else
                MaxDist = 1;

            if (searchHr.Count > 0)
            {
                HREntry hre = (HREntry)searchHr[0];
                searchHr.RemoveAt(0);

                furtherHr = hre.Rect;
                furtherKd = hre.Tree;
                pivot = hre.Pivot;
            }

            // step 10
            searchSteps -= 1;
            if (searchSteps > 0 && furtherHr.IsInReach(target, MaxDist))
            {
                float ptDist = GetDistance(pivot, target);
                if (ptDist < Distance)
                {
                    // steps 10.1.1 to 10.1.3
                    nearest = pivot;
                    Distance = ptDist;
                    MaxDist = Distance;
                }

                // step 10.2
                float tempDistSq;
                float[] tempNearest = new float[66];
                if (furtherKd == null)
                {
                    tempDistSq = 1;
                }
                else
                {
                    tempNearest = furtherKd.NearestNeighbourListBBFI(best, target, furtherHr, MaxDist, out tempDistSq, searchHr, ref searchSteps);
                }

                // step 10.3
                if (tempDistSq < Distance)
                {
                    nearest = tempNearest;
                    Distance = tempDistSq;
                }
            }

            resDistSq = Distance;
            return (nearest);
        }


        private float GetDistance(float[] t1, float[] t2)
        {
            float distance = 0;

            for (int n = 0; n < 64; ++n)
            {
                distance += (t1[n] - t2[n]) * (t1[n] - t2[n]);
            }
            return (float)Math.Sqrt(distance);
        }

         private static KDTree GoodCandidate(float[,] KPs, int Size)
        {
            KDTree Tree = new KDTree
            {
                KP = new float[66]
            };
            // initialize temporary hr search min/max values
            double[] minHr = new double[64];
            double[] maxHr = new double[64];
            for (int k = 0; k < 64; ++k)
            {
                minHr[k] = 1;
                maxHr[k] = -1;
            }
            for (int i = 0; i < Size; i++)
            {
                for (int k = 0; k < 64; ++k)
                {
                    double dimE = KPs[i, k];

                    if (dimE < minHr[k])
                        minHr[k] = dimE;
                    if (dimE > maxHr[k])
                        maxHr[k] = dimE;
                }
            }

            // find the maximum range dimension
            double[] diffHr = new double[64];
            int maxDiffDim = 0;
            double maxDiff = 0.0;
            for (int k = 0; k < 64; k++)
            {
                diffHr[k] = maxHr[k] - minHr[k];
                if (diffHr[k] > maxDiff)
                {
                    maxDiff = diffHr[k];
                    maxDiffDim = k;
                }
            }

            // the splitting dimension is maxDiffDim
            // now find a exemplar as close to the arithmetic middle as possible
            double middle = (maxDiff / 2.0) + minHr[maxDiffDim];

            double exemMinDiff = 1;

            for (int i = 0; i < Size; i++)
            {
                double curDiff = Math.Abs(KPs[i, maxDiffDim] - middle);
                if (curDiff < exemMinDiff)
                {
                    exemMinDiff = curDiff;

                    for (int j = 0; j < 66; j++)
                    {
                        Tree.KP[j] = KPs[i, j];
                    }
                }
            }

            Tree.splitDim = maxDiffDim;

            return (Tree);
        }


        // Build a kd-tree from a list of elements. All elements must implement
        // the IKDTreeDomain interface and must have the same dimensionality.
        public static KDTree CreateKDTree(float[,] KeypointList)
        {
            if (KeypointList == null)
                return (null);

            int Size = KeypointList.GetLength(0);

            KDTree Node = GoodCandidate(KeypointList, Size);


            // split the exemplar set into left/right elements relative to the
            // splitting dimension
            double bound = Node.KP[Node.splitDim];

            int LeftSize = 0, RightSize = 0, IngnorSize = 0;
            for (int i = 0; i < KeypointList.GetLength(0); i++)

            {
                // ignore the current element
                if ((KeypointList[i, 64] == Node.KP[64]) && (KeypointList[i, 65] == Node.KP[65]) )
                {
                    IngnorSize++;
                    continue;
                }
                if (KeypointList[i, Node.splitDim] <= bound)
                {
                    LeftSize++;
                }
            }
            RightSize = Size - LeftSize - IngnorSize;

            float[,] leftElems = null;
            float[,] rightElems = null;

            if (LeftSize != 0)
            {
                leftElems = new float[LeftSize, 66];
                LeftSize = 0;
            }
    
            if (RightSize != 0)
            {
                rightElems = new float[RightSize, 66];
                RightSize = 0;
            }
          
            for (int i = 0; i < Size; i++)
            {
                // ignore the current element
                if ((KeypointList[i, 64] == Node.KP[64]) && (KeypointList[i, 65] == Node.KP[65]) )
                    continue;

                if (KeypointList[i, Node.splitDim] <= bound)
                {
                    for (int m = 0; m < 66; m++)
                    {
                        leftElems[LeftSize, m] = KeypointList[i, m];
                    }
                    LeftSize++;
                }
                else
                {
                    for (int m = 0; m < 66; m++)
                    {
                        rightElems[RightSize, m] = KeypointList[i, m];
                    }
                    RightSize++;
                }
            }

            // recurse
            Node.Left = CreateKDTree(leftElems);
            Node.Right = CreateKDTree(rightElems);

            return (Node);
        }


       
    }


}