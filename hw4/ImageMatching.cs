using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.Windows;
using LOC.Image;
using LOC.Photogrammetry;
using LOC.Photogrammetry.CoordinateTransform;
using NSURF;

namespace LOC
{
    namespace FeatureMatching
    {
        public partial class ImageMatching
        {
            public partial class SURF
            {
                public SURF_Scale Scale;
                public int NofRefKeys, NofTarKeys;
                private int[] Filter;
                private int Dx = 0, Dy = 0;
                private readonly float Thres;
                private readonly bool IsExtend;
                private readonly bool IsUpRight;
                private InOrietation RefIO, TarIO;
                public Point[] RefPoly, TarPoly;
                public List<ImagePoints.Coordinate> RefMatchPTs = new List<ImagePoints.Coordinate>();
                public List<ImagePoints.Coordinate> TarMatchPTs = new List<ImagePoints.Coordinate>();
                public Dictionary<ImagePoints.Coordinate, ImagePoints.Coordinate> CleanMatchPairs = new Dictionary<ImagePoints.Coordinate, ImagePoints.Coordinate>();
                public Dictionary<ImagePoints.Coordinate, ImagePoints.Coordinate> RansacMatchPairs = new Dictionary<ImagePoints.Coordinate, ImagePoints.Coordinate>();
                public ConcurrentDictionary<ImagePoints.Coordinate, ImagePoints.Coordinate> InitialMatchPairs = new ConcurrentDictionary<ImagePoints.Coordinate, ImagePoints.Coordinate>();
                public float[,] RefKPs;
                public float[,] TarKPs;
                public bool IsGPU = false;
                public ImageMatchModel Matchmodel;
                private List<Match> MatchpointAL = new List<Match>();
               
                public SURF()
                { }

                public SURF(int NofRefKeys, int NofTarKeys, SURF_Scale Scale, bool IsUpRight, bool IsExtend, float Thres)
                {
                    this.NofRefKeys = NofRefKeys;
                    this.NofTarKeys = NofTarKeys;
                    this.IsUpRight = IsUpRight;
                    this.IsExtend = IsExtend;
                    this.Thres = Thres;
                    this.Scale = Scale;
                }


             

                public void Clear()
                {
                    CleanMatchPairs.Clear();
                    InitialMatchPairs.Clear();
                    RefMatchPTs.Clear();
                    TarMatchPTs.Clear();
                    MatchpointAL.Clear();

                    RefKPs = null;
                    TarKPs = null;                    
                    Matchmodel = null;
                }

            }
        }
    }
 
}






