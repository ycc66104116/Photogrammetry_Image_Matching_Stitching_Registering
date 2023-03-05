using System;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Single;
using LOC.Photogrammetry;

namespace NSURF
{
    public class RANSAC
    {
        public List<ImageMatchModel> FindModels(ImageMatchModel Model, List<Match> Matches, float Threshold)
        {
            Random Rand = new Random();
            List<ImageMatchModel> Result = new List<ImageMatchModel>();
            List<Match> Samples, InLiner;

            if (Matches.Count < 30)
                throw (new ArgumentException("List of data is smaller than minimum fit requires."));

            for (int i = 0; i < 30; ++i)
            {
                Samples = new List<Match>();

                do
                {
                    Match SampleToAdd;
                    SampleToAdd = Matches[Rand.Next(0, Matches.Count)];

                    if (Samples.Contains(SampleToAdd))
                        continue;

                    Samples.Add(SampleToAdd);
                } while (Samples.Count < 50);

                Model.FitModel(Samples);

                InLiner = new List<Match>();
                float SumFittingError = 0.0f;

                // Check all non-sample points for fit.
                foreach (Match MatchPoint in Matches)
                {
                    if (Samples.Contains(MatchPoint))
                        continue;

                    float Error = Model.FittingError(MatchPoint);

                    if (Error < Threshold)
                    {
                        InLiner.Add(MatchPoint);
                        SumFittingError += Error;
                    }
                }


                if (InLiner.Count >= 50)
                {
                    InLiner.AddRange(Samples);

                    Model = new ImageMatchModel(Threshold)
                    {
                        FittingErrorSum = SumFittingError / InLiner.Count,
                        FittingGround = InLiner
                    };

                    Result.Add(Model);
                }
            }
            Result.Sort();

            Samples = null;
            InLiner = null;

            return (Result);
        }
    }
    public class ImageMatchModel : IComparable
    {

        private float Thresh;
        public float FittingErrorSum;
        public List<Match> FittingGround;
        public float[] Coefs = new float[8];


        public ImageMatchModel(float Thresh)
        {
            this.Thresh = Thresh;
        }

        public void FitModel(List<Match> Matches)
        {
            List<ImagePoints.Coordinate> Tar = new List<ImagePoints.Coordinate>();
            List<ImagePoints.Coordinate> Ref = new List<ImagePoints.Coordinate>();

            for (int i = 0; i < Matches.Count; i++)
            {
                Ref.Add(new ImagePoints.Coordinate(Matches[i].Kp1.X, Matches[i].Kp1.Y, LOC.CoordinateFormat.Pixel));
                Tar.Add(new ImagePoints.Coordinate(Matches[i].Kp2.X, Matches[i].Kp2.Y, LOC.CoordinateFormat.Pixel));
            }

            Coefs = ProjectiveTranaform(Ref, Tar);
        }

        public float FittingError(Match Match)
        {
            float ExpectX = 0, ExpectY = 0;
            ExpectX = (Coefs[0] * Match.Kp1.X + Coefs[1] * Match.Kp1.Y + Coefs[2]) / (Coefs[6] * Match.Kp1.X + Coefs[7] * Match.Kp1.Y + 1);
            ExpectY = (Coefs[3] * Match.Kp1.X + Coefs[4] * Match.Kp1.Y + Coefs[5]) / (Coefs[6] * Match.Kp1.X + Coefs[7] * Match.Kp1.Y + 1);

            return (float)Math.Sqrt(Math.Pow(ExpectX - Match.Kp2.X, 2.0) + Math.Pow(ExpectY - Match.Kp2.Y, 2.0));

        }

      
        public static float[] ProjectiveTranaform(List<ImagePoints.Coordinate> Tar, List<ImagePoints.Coordinate> Ref)
        {
            float[] Coefs = new float[8];
            LeastSquare LSA = new LeastSquare();

            do
            {
                int Points = Tar.Count;

                Matrix<float> A = DenseMatrix.CreateDiagonal(Points * 2, 8, 1); //建立A矩陣，預設值為1                                                                                                  
                Matrix<float> L = DenseMatrix.CreateDiagonal(Points * 2, 1, 1);

                for (int i = 0; i < Points; i++)
                {
                    int Index_X = i * 2;
                    int Index_Y = Index_X + 1;

                    A[Index_X, 0] = A[Index_Y, 3] = Tar[i].X;
                    A[Index_X, 1] = A[Index_Y, 4] = Tar[i].Y;
                    A[Index_X, 2] = A[Index_Y, 5] = 1;
                    A[Index_X, 3] = A[Index_X, 4] = A[Index_X, 5] = A[Index_Y, 0] = A[Index_Y, 1] = A[Index_Y, 2] = 0;

                    A[Index_X, 6] = -Tar[i].X * Ref[i].X;
                    A[Index_X, 7] = -Tar[i].Y * Ref[i].X;
                    A[Index_Y, 6] = -Tar[i].X * Ref[i].Y;
                    A[Index_Y, 7] = -Tar[i].Y * Ref[i].Y;

                    L[Index_X, 0] = Ref[i].X;
                    L[Index_Y, 0] = Ref[i].Y;
                }

                LSA.Adjustment(A, L);

            } while (BlunderRemoval.RemoveBlunder(ref Tar, ref Ref, LSA, 2.2f) == true);


            Coefs[0] = LSA.X[0, 0];
            Coefs[1] = LSA.X[1, 0];
            Coefs[2] = LSA.X[2, 0];
            Coefs[3] = LSA.X[3, 0];
            Coefs[4] = LSA.X[4, 0];
            Coefs[5] = LSA.X[5, 0];
            Coefs[6] = LSA.X[6, 0];
            Coefs[7] = LSA.X[7, 0];

            return (Coefs);
        }

        struct BlunderRemoval
        {
            public static bool RemoveBlunder(ref List<ImagePoints.Coordinate> Original, ref List<ImagePoints.Coordinate> Transform, LeastSquare LSA, float BlunderOrder)
            {
                int Points = Original.Count;
                int Size = 0;
                const int Equation = 2;
                bool IsBlunders = false;

                float Upbound = LSA.Mean + BlunderOrder * LSA.RMSE;
                float Lowbound = LSA.Mean - BlunderOrder * LSA.RMSE;
                Size = Points * Equation - 1;
                int RemoveCounts = 0;

                for (int i = Size; i >= 0; i = i - Equation)
                {
                    if (LSA.V[i, 0] > Upbound || LSA.V[i, 0] < Lowbound || LSA.V[i - 1, 0] > Upbound || LSA.V[i - 1, 0] < Lowbound)
                    {
                        Original.RemoveAt(i / Equation);
                        Transform.RemoveAt(i / Equation);
                        IsBlunders = true;
                        RemoveCounts++;
                    }
                }
                return IsBlunders;
            }

        }



        public int CompareTo(object obj)
        {
            ImageMatchModel mod = (ImageMatchModel)obj;

            if (FittingErrorSum < mod.FittingErrorSum)
                return (-1);
            else if (FittingErrorSum > mod.FittingErrorSum)
                return (1);

            return (0);
        }
    }

    public class MatchDriver
    {
        public static ImageMatchModel FilterMatchSet(List<Match> Matches, float Threshold)
        {
            if (Matches.Count <= 20)
                return (null);

            RANSAC Ransac = new RANSAC();

            ImageMatchModel Mod = new ImageMatchModel(Threshold);

            List<ImageMatchModel> Models = Ransac.FindModels(Mod, Matches, Threshold);

            if (Models.Count == 0)
                return (null);

            if (Models[0].Coefs[0] == 0)
            {
                Mod = Models[1];
            }
            else
            {
                Mod = Models[0];
            }

            return (Mod);
        }
    }


}