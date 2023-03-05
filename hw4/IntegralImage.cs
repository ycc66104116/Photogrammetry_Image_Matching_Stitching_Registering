using System;
using System.Windows;
using LOC.Image;

namespace NSURF
{
    public struct IntegralImage
    {
        internal float[] Matrix;
        internal int Width, Height,Size,  OffsetWidth, OffsetHeight;

     
        private IntegralImage(LOCImage Image)
        {
            Width = Image.Width;
            Height = Image.Height;
            Size = Width * Height;
            Matrix = new float[Height* Width];
            OffsetWidth = Image.Off_W;
            OffsetHeight = Image.Off_H;
            

        }
        private IntegralImage(LOCImage Image, Int32Rect Rect)
        {

            Width = Rect.Width;
            Height = Rect.Height;
            Size = Width * Height;
            Matrix = new float[Size];
            OffsetWidth = Rect.X;
            OffsetHeight = Rect.Y;
           
        }

        public static IntegralImage FromImage(LOCImage Image, Int32Rect Rect)
        {
            IntegralImage pic = new IntegralImage(Image, Rect);
            float rowsum = 0;
            int Width_Range = Rect.X + Rect.Width, Height_Range = Rect.Y + Rect.Height;
            int SelectBand = 0;
            int Img_Idx = 0, Index = 0;
            if (Image.NoBands == 3 || Image.NoBands == 4)
            {
                SelectBand = 1;
            }

            if (Image.NoBytes == 1)
            {
                for (int x = Rect.X; x < Width_Range; x++)
                {
                    Img_Idx = x * Image.ByteBands + SelectBand;
                    Index = (x - Rect.X) ;
                    rowsum += (Image.ByteData[Img_Idx]);
                    pic.Matrix[Index] = rowsum;
                }
            }
            else if (Image.NoBytes == 2)
            {
                for (int x = Rect.X; x < Width_Range; x++)
                {
                    Img_Idx = x * Image.ByteBands + SelectBand;
                    Index = (x - Rect.X);
                    rowsum += (Image.ByteData[Img_Idx] / 256 + Image.ByteData[Img_Idx + 1]);
                    pic.Matrix[Index] = rowsum;
                }
            }


            if (Image.NoBytes == 1)
            {
                for (int y = 1 + Rect.Y; y < Height_Range; y++)
                {
                    rowsum = 0;
                    for (int x = Rect.X; x < Width_Range; x++)
                    {
                        Img_Idx = (y * Image.Width + x) * Image.NoBands + SelectBand;
                        Index = ((y - Rect.Y) * Rect.Width + x - Rect.X) ;
                        rowsum += (Image.ByteData[Img_Idx]);
                        // integral image is rowsum + value above        
                        pic.Matrix[Index] = rowsum + pic.Matrix[Index - Rect.Width];
                    }
                }
            }
            else if (Image.NoBytes == 2)
            {
                for (int y = 1 + Rect.Y; y < Height_Range; y++)
                {
                    rowsum = 0;
                    for (int x = Rect.X; x < Width_Range; x++)
                    {
                        Img_Idx = (y * Image.Width + x) * Image.ByteBands;
                        Index = ((y - Rect.Y) * Rect.Width + x - Rect.X) ;
                        rowsum += (Image.ByteData[Img_Idx] / 256 + Image.ByteData[Img_Idx + 1]);
                        // integral image is rowsum + value above        
                        pic.Matrix[Index] = rowsum + pic.Matrix[Index - Rect.Width];
                    }
                }
            }
            return pic;
        }

        public static IntegralImage FromImage(LOCImage Image)
        {
            IntegralImage pic = new IntegralImage(Image);
            float rowsum = 0;

            int SelectBand = 0;
            int Index = 0;
            if (Image.NoBands == 3 || Image.NoBands == 4)
            {
                SelectBand = 1;
            }

            if (Image.NoBytes == 1)
            {
                for (int x = 0; x < Image.Width; x++)
                {
                    Index = x * Image.NoBands + SelectBand;
                    rowsum += (Image.ByteData[Index]);
                    pic.Matrix[x] = rowsum;
                }
            }
            else if (Image.NoBytes == 2)
            {
                for (int x = 0; x < Image.Width; x++)
                {
                    Index = x * Image.ByteBands;
                    rowsum += (Image.ByteData[Index] / 256 + Image.ByteData[Index + 1]);
                    pic.Matrix[x] = rowsum;
                }
            }


            if (Image.NoBytes == 1)
            {
                for (int y = 1; y < Image.Height; y++)
                {
                    rowsum = 0;
                    for (int x = 0; x < Image.Width; x++)
                    {
                        Index = (y * Image.Width + x) * Image.NoBands + SelectBand;
                        rowsum += (Image.ByteData[Index]);
                        // integral image is rowsum + value above        
                        pic.Matrix[(y * Image.Width + x)] = rowsum + pic.Matrix[((y - 1) * Image.Width + x)];
                    }
                }
            }
            else if (Image.NoBytes == 2)
            {
                for (int y = 1; y < Image.Height; y++)
                {
                    rowsum = 0;
                    for (int x = 0; x < Image.Width; x++)
                    {
                        Index = (y * Image.Width + x) * Image.ByteBands;
                        rowsum += (Image.ByteData[Index] / 256 + Image.ByteData[Index + 1]);
                        // integral image is rowsum + value above        
                        pic.Matrix[(y * Image.Width + x)] = rowsum + pic.Matrix[((y - 1) * Image.Width + x)];
                    }
                }
            }
            return pic;
        }


 

        public static float BoxIntegral(int row, int col, int rows, int cols, int Width, int Height, float[] Matrix)
        {
            // The subtraction by one for row/col is because row/col is inclusive.

            float A = 0, B = 0, C = 0, D = 0;
            int Index1 = 0, Index2=0, Index3=0,Index4=0;
            if (row == Int32.MaxValue  || col == Int32.MaxValue )
                return 0;


            int r1 = Math.Min(row, Height) - 1;
            int c1 = Math.Min(col, Width) - 1;
            int r2 = Math.Min(rows, Height) - 1;
            int c2 = Math.Min(cols, Width) - 1;

            Index1 = r1 * Width + c1;
            Index2 = r1 * Width + c2;
            Index3 = r2 * Width + c1;
            Index4 = r2 * Width + c2;


            if (r1 >= 0 && c1 >= 0) A = Matrix[Index1];
            if (r1 >= 0 && c2 >= 0) B = Matrix[Index2];
            if (r2 >= 0 && c1 >= 0) C = Matrix[Index3];
            if (r2 >= 0 && c2 >= 0) D = Matrix[Index4];

            return Math.Max(0, A - B - C + D);
        }
        public static float HaarX(int row, int column, int size, int size2, int Width, int Height, float[] Matrix)
        {

            return BoxIntegral(row - size2, column, row + size, column + size2, Width, Height, Matrix) - BoxIntegral(row - size2, column - size2, row + size, column, Width, Height, Matrix);
        }

        public static float HaarY(int row, int column, int size, int size2, int Width, int Height, float[] Matrix)
        {
            return BoxIntegral(row, column - size2, row + size2, column + size, Width, Height, Matrix) - BoxIntegral(row - size2, column - size2, row, column + size, Width, Height, Matrix);
        }

    }
}
