using System.Linq;
using System.Threading.Tasks;
using System.Windows;

namespace NSURF
{
    public partial struct FastHessian
    {
        private IntegralImage Img;
        private KeyPoint[] KeyPoints;
        private float[] Matrix;
        private int Width, Height, Size, NoF, NoKPs;
        private int[,] ThresCum;
        private float[,] Responses;
        private int[,] ExtremumData;

        public static KeyPoint[] GetKeyPoints(IntegralImage Img, int Keypoints, int[] Filter)
        {
            FastHessian fh = new FastHessian
            {
                Img = Img,
                Width = Img.Width,
                Height = Img.Height,
                Size = Img.Size,
                Matrix = Img.Matrix,

                NoF = Filter.GetLength(0),
            };
            fh.ExtremumData = new int[fh.NoF, fh.Size * 4];
            fh.ThresCum = new int[fh.NoF, 100001];
            return fh.GetIpoints(Keypoints, Filter);
        }


        private KeyPoint[] GetIpoints(int Keys, int[] Filter)
        {
            int Thres = 0;

           
                  BuildResponseLayer(Width, Height, NoF,Size, Filter,Matrix);
            
            IsExtremum();

            for (int n = 0; n < NoF; n++)
            {
                Thres = 0;
                if (Keys == int.MaxValue)
                    Thres = 0;
                else
                {
                    for (int CDF = 99999; CDF >= 0; CDF--)
                    {
                        ThresCum[n, CDF] = ThresCum[n, CDF] + ThresCum[n, CDF + 1];
                        if (ThresCum[n, CDF] - Keys >= 0 && Thres == 0)
                        {
                            Thres = CDF;
                            break;
                        }
                        else
                        {
                            Thres = 0;
                        }
                    }
                }
                NoKPs += ThresCum[n, Thres];
            }

            KeyPoints = new KeyPoint[NoKPs];

            int Count = 0, Size4;

            for (int n = 0; n < NoF; n++)
            {
                int Scale = 1;
                if (Filter[n] == 9)
                {
                    Scale = 1;
                }
                if (Filter[n] == 15)//1.2*15/9
                {
                    Scale = 2;
                }
                if (Filter[n] == 21)// 1.2*21/9
                {
                    Scale = 3;
                }
                if (Filter[n] == 27) // 1.2*27/9
                {
                    Scale = 4;
                }
                if (Filter[n] == 33) // 1.2*33/9
                {
                    Scale = 5;
                }


                for (int j = 0; j < Size; j++)
                {
                    Size4 = j * 4;
                    if (ExtremumData[n, Size4 + 2] != 0 && ExtremumData[n, Size4 + 0] != 0 && ExtremumData[n, Size4 + 1] != 0 && ExtremumData[n, Size4 + 3] >= Thres)
                    {
                        if (Count <NoKPs)
                        {
                            InterpolateExtremum(ExtremumData[n, Size4 + 1], ExtremumData[n, Size4], ExtremumData[n, Size4 + 2], Responses, n, Count, Scale);
                            Count++;
                        }
                    }
                }
            }


            return KeyPoints;
        }

        private static bool IsInPolygon(Point[] Poly, Point Pt)
        {
            var Coef = Poly.Skip(1).Select((P, i) => (Pt.Y - Poly[i].Y) * (P.X - Poly[i].X) - (Pt.X - Poly[i].X) * (P.Y - Poly[i].Y)).ToList();

            if (Coef.Any(p => p == 0))
                return true;

            for (int i = 1; i < Coef.Count(); i++)
            {
                if (Coef[i] * Coef[i - 1] < 0)
                    return false;
            }
            return true;
        }


        /// <summary>
        /// Build Responses for a given ResponseLayer
        /// </summary>
        /// <param name="ResponseLayer"></param>
        private void BuildResponseLayer(int Width, int Height, int NoF, int Size, int[] Filter, float[] Matrix)
        {
            float[,] Responses = new float[NoF, Size];
            for (int n = 0; n < NoF; n++)
            {
                int B = (Filter[n] - 1) / 2;             // border for this filter
                int L = Filter[n] / 3;                   // lobe for this filter (filter size / 3)
                int W = Filter[n];
                float inverse_are1 = 1f / (W * W);
                float inverse_are2 = inverse_are1 * 0.81f;       // normalisation factor
              

                Parallel.For(0, Size, Index =>
                {
                    float Dxx = 1, Dyy = 1, Dxy = 1;
                    int r = 0, c = 0;
                    r = Index / Width;
                    c = Index % Width;

                    Dxx = IntegralImage.BoxIntegral(r - L + 1, c - B, r + L, c - B + W, Width, Height, Matrix)
                        - IntegralImage.BoxIntegral(r - L + 1, c - L / 2, r + L, c - L / 2 + L, Width, Height, Matrix) * 3;
                    Dyy = IntegralImage.BoxIntegral(r - B, c - L + 1, r - B + W, c + L, Width, Height, Matrix)
                        - IntegralImage.BoxIntegral(r - L / 2, c - L + 1, r - L / 2 + L, c + L, Width, Height, Matrix) * 3;
                    Dxy = +IntegralImage.BoxIntegral(r - L, c + 1, r, c + 1 + L, Width, Height, Matrix)
                          + IntegralImage.BoxIntegral(r + 1, c - L, r + 1 + L, c, Width, Height, Matrix)
                          - IntegralImage.BoxIntegral(r - L, c - L, r, c, Width, Height, Matrix)
                          - IntegralImage.BoxIntegral(r + 1, c + 1, r + 1 + L, c + 1 + L, Width, Height, Matrix);

                    Responses[n,Index] = (Dxx * Dyy * inverse_are1 - Dxy * Dxy * inverse_are2);
                });

                for (int i = 0; i < Size; i++)
                {
                    if (Responses[n,i] > 100000)
                    {
                        Responses[n,i] = 100000;
                    }
                }
            }
            this.Responses = Responses;

        }

   
        private int[,] IsExtremum()
        {
            int Width = Img.Width, Height = Img.Height;
            int Index = 0, Index4 = 0, IndexMinus, IndexPlus;
            float[] Neighbor = new float[9];
            for (int n = 0; n < NoF; n++)
            {
                for (int c = 1; c < Width - 1; c++)
                {
                    for (int r = 1; r < Height - 1; r++)
                    {
                        Index = r * Width + c;
                        Index4 = Index * 4;
                        IndexMinus = Index - Width;// (r - 1) * Width + c;
                        IndexPlus = Index + Width;// (r + 1) * Width + c;

                        Neighbor[0] = Responses[n, IndexMinus - 1];
                        Neighbor[1] = Responses[n, IndexMinus];
                        Neighbor[2] = Responses[n, IndexMinus + 1];
                        Neighbor[3] = Responses[n, Index - 1];
                        Neighbor[4] = Responses[n, Index];
                        Neighbor[5] = Responses[n, Index + 1];
                        Neighbor[6] = Responses[n, IndexPlus - 1];
                        Neighbor[7] = Responses[n, IndexPlus];
                        Neighbor[8] = Responses[n, IndexPlus + 1];

                        if ((Neighbor[4] - Neighbor[0]) > 0 && (Neighbor[4] - Neighbor[1]) > 0 && (Neighbor[4] - Neighbor[2]) > 0 && (Neighbor[4] - Neighbor[3]) > 0 && (Neighbor[4] - Neighbor[5]) > 0 && (Neighbor[4] - Neighbor[6]) > 0 && (Neighbor[4] - Neighbor[7]) > 0 && (Neighbor[4] - Neighbor[8]) > 0 && Neighbor[4] > 0)
                        {
                            ExtremumData[n, Index4 + 0] = c;
                            ExtremumData[n, Index4 + 1] = r;
                            ExtremumData[n, Index4 + 2] = Index;
                            ExtremumData[n, Index4 + 3] = (int)(Neighbor[4]);
                            ThresCum[n, (int)(Neighbor[4])]++;
                        }
                    }
                }
            }
            return ExtremumData;
        }


        /// <summary>
        /// Interpolate scale-space extrema to subpixel accuracy to form an image feature
        /// </summary>
        /// <param name="r"></param>
        /// <param name="c"></param>
        /// <param name="t"></param>
        /// <param name="m"></param>
        /// <param name="b"></param>
        void InterpolateExtremum(int r, int c, int Index, float[,] Responses, int N, int Count, int Scale)
        {
            float[] Of = new float[2];
            Of = DerivativeHessian(Index, N, Responses);
          
            if (Of == null|| float.IsNaN(Of[0]) || c == 0 )
            {
                return;
            }
            KeyPoint ipt = new KeyPoint
            {
                X = c + Of[0] + Img.OffsetWidth,
                Y = r + Of[1] + Img.OffsetHeight,
                IntX = c,
                IntY = r,
                Scale = Scale,
            };

            KeyPoints[Count] = ipt;
        }

        private float[] DerivativeHessian(int Index, int N, float[,] Responses)
        {
            float det = 0;
            int IndexPlus = Index + Img.Width;    //(r+1,c)
            int IndexMinus = Index - Img.Width;   //(r-1,c)
            float v = Responses[N, Index] * 2;
            float m1 = Responses[N, Index + 1] - Responses[N, Index - 1];
            float m2 = Responses[N, IndexPlus] - Responses[N, IndexMinus];
            float[] D = new float[2];
            float[,] H = new float[2, 2];

            D[0] = m1 / 2f;
            D[1] = m2 / 2f;

            H[0, 0] = Responses[N, Index + 1] + Responses[N, Index - 1] - v;
            H[1, 1] = Responses[N, IndexPlus] + Responses[N, IndexMinus] - v;
            H[0, 1] = H[1, 0] = (Responses[N, IndexPlus + 1] - Responses[N, IndexPlus - 1] - Responses[N, IndexMinus + 1] + Responses[N, IndexMinus - 1]) / 4f;

            det = H[0, 0] * H[1, 1] - H[0, 1] * H[1, 0];
            if (det == 0)
            {
                return null;
            }
            float[] O = new float[2];

            O[0] = -H[1, 1] / det * D[0] + H[0, 1] / det * D[1];
            O[1] = H[1, 0] / det * D[0] - H[0, 0] / det * D[1];

            return O;
        }

    } // FastHessian
}

