using System;
using System.Threading.Tasks;
using System.Collections.Generic;

namespace NSURF
{
    public class SurfDescriptor
    {
    
        IntegralImage Img;
        const float Pi  = 3.1415926535897931f;
        const float Pi2 = 6.2831853071795862f;
        const float Pi_3 = 1.047197551f;
        const float Pi5_3 = 5.235987756f;
        const byte Responses = 109;
        const int Dim = 66;
        int Size;
        

        /// <summary>
        /// Static one-call do it all function
        /// </summary>
        /// <param name="img"></param>
        /// <param name="ipts"></param>
        /// <param name="extended"></param>
        /// <param name="upright"></param>
        public static float[,] DecribePoints(KeyPoint[] KPs, IntegralImage img, bool UpRight, bool IsExtended)
        {
            SurfDescriptor des = new SurfDescriptor();
            return des.DescribePoints(KPs, img, UpRight, IsExtended);
        }


        /// <summary>
        /// Build descriptor vector for each interest point in the supplied list
        /// </summary>
        /// <param name="img"></param>
        /// <param name="ipts"></param>
        /// <param name="upright"></param>
        public float[,] DescribePoints(KeyPoint[] KPs, IntegralImage Img, bool UpRight, bool IsExtended)
        {
            this.Img = Img;
            Size = KPs.GetLength(0);
            float[] Matrix = Img.Matrix;
            if (!UpRight)
            {

                GetOrientation(KPs, Matrix);

            }
            float[,] Descripotor = new float[Size, Dim];

            return Descripotor = GetDescriptor(KPs, UpRight, IsExtended, Matrix);

        }


        private void GetOrientation(KeyPoint[] KP, float[] Matrix)
        {           
            int Width = Img.Width, Height = Img.Height;            
            float[,,] Haar = new float[3, Size, Responses];     

            Parallel.For(0, Size, i =>
            {
                float sumX = 0, sumY = 0, max = 0, ang1 = 0, ang2 = 0;
                double orientation = 0;
                int idx = 0,  S = KP[i].Scale, S2 = S * 2, S4 = S * 4;                
                int X = KP[i].IntX, Y = KP[i].IntY;
                //calculate haar responses for points within radius of 6 * scale
                for (int m = -6; m <= 6; ++m)
                {
                    for (int n = -6; n <= 6; ++n)
                    {
                        if (m * m + n * n < 36)
                        {                         
                            Haar[0, i, idx] =  IntegralImage.HaarX(Y + n * S, X + m * S, S4, S2, Width, Height, Matrix);
                            Haar[1, i, idx] =  IntegralImage.HaarY(Y + n * S, X + m * S, S4, S2, Width, Height, Matrix);
                            Haar[2, i, idx] = (float)GetAngle(Haar[0, i, idx], Haar[1, i, idx]);
                            ++idx;
                        }
                    }
                }
                // calculate the dominant direction                            
                // loop slides pi/3 window around feature point

                for (ang1 = 0; ang1 < Pi2; ang1 += 0.15f)
                {
                    ang2 = (ang1 + Pi_3 > Pi2 ? ang1 - Pi5_3 : ang1 + Pi_3);
                    sumX = sumY = 0;

                    for (int k = 0; k < Responses; ++k)
                    {
                        // determine whether the point is within the window
                        if (ang1 < ang2 && ang1 < Haar[2, i, k] && Haar[2, i, k] < ang2)
                        {
                            sumX += Haar[0, i, k];
                            sumY += Haar[1, i, k];
                        }
                        else if (ang2 < ang1 && ((Haar[2, i, k] > 0 && Haar[2, i, k] < ang2) || (Haar[2, i, k] > ang1 && Haar[2, i, k] < Pi)))
                        {
                            sumX += Haar[0, i, k];
                            sumY += Haar[1, i, k];
                        }
                    }

                    // if the vector produced from this window is longer than all 
                    //// previous vectors then this forms the new dominant direction

                    if (sumX * sumX + sumY * sumY > max)
                    {
                        // store largest orientation
                        max = sumX * sumX + sumY * sumY;
                        orientation = GetAngle(sumX, sumY);
                    }
                }
                // assign orientation of the dominant response vector

                KP[i].Orientation = orientation;

            });
        }


        private float[,] GetDescriptor(KeyPoint[] KP, bool UpRight, bool IsExtended, float[] Matrix )
        {
            int OffsetWidth = Img.OffsetWidth, OffsetHeight = Img.OffsetHeight, Width = Img.Width, Height = Img.Height;
          
            if (IsExtended)
            {
               // Dim = 128;
            }
            float[,] Descripotor = new float[Size, Dim];
      
            Parallel.For(0, Size, m =>
            {
                float dx = 0, dy = 0, mdx = 0, mdy = 0, co = 0, si = 0;
                float dx_yn = 0, mdx_yn = 0, dy_xn = 0, mdy_xn = 0;
                float rx = 0f, ry = 0f, rrx = 0f, rry = 0f, len = 0f;
                int count = 0, sample_x = 0, sample_y = 0, count4 = 0;

                int X = (KP[m].IntX);
                int Y = (KP[m].IntY );
                int S = KP[m].Scale, S2 = S * 2;

                Descripotor[m, 64] = KP[m].X;
                Descripotor[m, 65] = KP[m].Y;
               
                if (UpRight)
                {
                    co = 1;
                    si = 0;
                }
                else
                {
                    co = (float)Math.Cos(KP[m].Orientation);
                    si = (float)Math.Sin(KP[m].Orientation);
                }

                for (int i = -12; i < 8; i = i + 5)
                {                
                    for (int j = -12; j < 8; j = j + 5)
                    {
   
                        // zero the responses
                        dx = dy = mdx = mdy =  dx_yn = mdx_yn = dy_xn = mdy_xn = 0f;

                        for (int k = i; k < i + 9; ++k)
                        {
                            for (int l = j; l < j + 9; ++l)
                            {
                                //Get coords of sample point on the rotated axis
                                if (UpRight)
                                {
                                    sample_x = (X + k * S);
                                    sample_y = (Y + l * S);
                                   
                                }
                                else
                                {
                                    sample_x = (int)(X + -l * S * si + k * S * co);
                                    sample_y = (int)(Y + l * S * co + k * S * si);
                                }
                               
                                //Get the gaussian weighted x and y responses
             
                                rx = IntegralImage.HaarX(sample_y, sample_x, S2, S, Width, Height, Matrix);
                                ry = IntegralImage.HaarY(sample_y, sample_x, S2, S, Width, Height, Matrix);

                                //  //Get the gaussian weighted x and y responses on rotated axis
                                if (UpRight)
                                {                                
                                    rrx = ry;
                                    rry = rx;
                                }
                                else
                                {                                   
                                    rrx = (-rx * si + ry * co);
                                    rry = (rx * co + ry * si);
                                }


                                if (IsExtended)
                                {
                                    // split x responses for different signs of y
                                    if (rry >= 0)
                                    {
                                        dx += rrx;
                                        mdx += Math.Abs(rrx);
                                    }
                                    else
                                    {
                                        dx_yn += rrx;
                                        mdx_yn += Math.Abs(rrx);
                                    }
                                    // split y responses for different signs of x
                                    if (rrx >= 0)
                                    {
                                        dy += rry;
                                        mdy += Math.Abs(rry);
                                    }
                                    else
                                    {
                                        dy_xn += rry;
                                        mdy_xn += Math.Abs(rry);
                                    }
                                }
                                else
                                {
                                    dx += rrx;
                                    dy += rry;
                                    mdx += Math.Abs(rrx);
                                    mdy += Math.Abs(rry);
                                }
                            }
                        }

                        //Add the values to the descriptor vector
                       
                        Descripotor[m, count4    ] = dx ;
                        Descripotor[m, count4 + 1] = dy ;
                        Descripotor[m, count4 + 2] = mdx ;
                        Descripotor[m, count4 + 3] = mdy ;

                        // add the extended components
                        if (IsExtended)
                        {
                            Descripotor[m, count4 + 4] = dx_yn ;
                            Descripotor[m, count4 + 5] = dy_xn ;
                            Descripotor[m, count4 + 6] = mdx_yn;
                            Descripotor[m, count4 + 7] = mdy_xn;
                        }

                        len += (dx * dx + dy * dy + mdx * mdx + mdy * mdy + dx_yn + dy_xn + mdx_yn + mdy_xn) ;
                        count++;
                        count4 = count * 4;
                    }
                }

                //Convert to Unit Vector
                len = (float)Math.Sqrt(len);
                if (len > 0)
                {
                    for (int d = 0; d < 64; d++)
                    {
                        Descripotor[m, d] = Descripotor[m, d] / len;
                    }
                }
            });

            return Descripotor;
        }


       

 

        private double GetAngle(float X, float Y)
        {
            if (X >= 0 && Y >= 0)
                return Math.Atan(Y / X);
            else if (X < 0 && Y >= 0)
                return Pi - Math.Atan(-Y / X);
            else if (X < 0 && Y < 0)
                return Pi + Math.Atan(Y / X);
            else if (X >= 0 && Y < 0)
                return Pi2 - Math.Atan(-Y / X);

            return 0;
        }

        //private float Gaussian(int x, int y, float sig)
        //{

        //    return (1f / (Pi2 * sig * sig)) * (float)Math.Exp(-(x * x + y * y) / (2.0f * sig * sig));
        //}
        private float Gaussian(float x, float y, float sig)
        {
            float sig2 = sig * sig;
            return 1f / (Pi2 * sig2) * (float)Math.Exp(-(x * x + y * y) / (2.0f * sig2));
        }


    } // SurfDescriptor


}
