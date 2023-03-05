using System;
using System.IO;
using System.Collections.Generic;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;

namespace LOC
{
    namespace Image
    {
        public class LOCImage : IDisposable
        {
            public string FileName;
            public int Width, Height, NoBytes, NoBands, Off_W, Off_H, ByteBands, Stride;
            public byte[] ByteData;
            public byte[,,] ByteData2D;
            public double DpiX, DpiY;
            public PixelFormat PixelFormat;
            public BitmapMetadata Metadata;
            public BitmapSource Source;

            private bool Disposed = false;


            public LOCImage()
            { }
            public LOCImage(int Width, int Height, double DpiX, double DpiY, PixelFormat PixelFormat, BitmapMetadata Metadata)
            {
                this.Width = Width;
                this.Height = Height;
                NoBands = PixelFormat.Masks.Count;
                NoBytes = (PixelFormat.BitsPerPixel / NoBands) / 8;
                ByteBands = NoBands * NoBytes;
                this.DpiX = DpiX;
                this.DpiY = DpiY;
                this.PixelFormat = PixelFormat;
                this.Metadata = Metadata;
                ByteData = new byte[Width * Height * NoBytes * NoBands];
            }
        



            /// <summary>
            /// 
            /// </summary>
            /// <param name="FileName"></param>
            public LOCImage(string FileName, Int32Rect Rect)
            {
                this.FileName = FileName;
                Open(FileName, Rect);
            }


      
            private void Open(string FileName, Int32Rect Rect)
            {
                Source = BitmapDecoder.Create(new Uri(FileName), BitmapCreateOptions.PreservePixelFormat, BitmapCacheOption.OnLoad).Frames[0];
                GetImageInfo(Source, Rect);

                GC.Collect();
            }


            public void Save(string FileName, ImageFormat ImageFormat)
            {
                int Stride = Width * NoBands * NoBytes;
                BitmapSource BitmapSource = BitmapSource.Create(Width, Height, DpiX, DpiY, PixelFormat, null, ByteData, Stride);

                using (var Stream = File.Create(FileName))
                {
                    switch (ImageFormat)
                    {
                        case ImageFormat.Jpeg:
                            JpegBitmapEncoder JpgImage = new JpegBitmapEncoder();
                            JpgImage.Frames.Add(BitmapFrame.Create(BitmapSource, null, Metadata, null));
                            JpgImage.QualityLevel = 90;
                            JpgImage.Save(Stream);
                            JpgImage = null;
                            break;

                        case ImageFormat.Tiff:
                            TiffBitmapEncoder TiffImage = new TiffBitmapEncoder();
                            TiffImage.Frames.Add(BitmapFrame.Create(BitmapSource, null, Metadata, null));
                            TiffImage.Compression = TiffCompressOption.None;
                            TiffImage.Save(Stream);
                            TiffImage = null;
                            break;

                        case ImageFormat.Bmp:
                            BmpBitmapEncoder BmpImage = new BmpBitmapEncoder();
                            BmpImage.Frames.Add(BitmapFrame.Create(BitmapSource, null, Metadata, null));
                            BmpImage.Save(Stream);
                            BmpImage = null;
                            break;

                        case ImageFormat.Png:
                            PngBitmapEncoder PngImage = new PngBitmapEncoder();
                            PngImage.Frames.Add(BitmapFrame.Create(BitmapSource, null, Metadata, null));
                            PngImage.Save(Stream);
                            PngImage.Interlace = PngInterlaceOption.On;
                            PngImage = null;
                            break;

                        case ImageFormat.Wmp:
                            WmpBitmapEncoder WmpImage = new WmpBitmapEncoder();
                            WmpImage.Frames.Add(BitmapFrame.Create(BitmapSource, null, Metadata, null));
                            WmpImage.Save(Stream);
                            WmpImage.Lossless = false;
                            WmpImage = null;
                            break;
                    }

                    BitmapSource = null;
                }

                GC.Collect();
            }

       
     
         

            private void GetImageInfo(BitmapSource Source, Int32Rect Rect)
            {
                NoBands = Source.Format.Masks.Count;
                NoBytes = (Source.Format.BitsPerPixel / NoBands) / 8;
                ByteBands = NoBands * NoBytes;
                DpiX = Source.DpiX;
                DpiY = Source.DpiY;
                PixelFormat = Source.Format;
                Metadata = (BitmapMetadata)Source.Metadata;

                if (Rect.IsEmpty == false)
                {
                    Width = Rect.Width;
                    Height = Rect.Height;
                    Off_W = Rect.X;
                    Off_H = Rect.Y;
                    Stride = Width * ByteBands;
                    ByteData = new byte[Stride * Height];
                    Source.CopyPixels(Rect, ByteData, Stride, 0);
                }
                else
                {
                    Width = Source.PixelWidth;
                    Height = Source.PixelHeight;
                    Off_W = Off_H = 0;
                    Stride = Width * ByteBands;
                    ByteData = new byte[Stride * Height];
                    Source.CopyPixels(ByteData, Stride, 0);
                }

                Source = null;
                GC.Collect();
            }


            public void Dispose()
            {
                Dispose(true);
                GC.SuppressFinalize(this);
            }

            protected virtual void Dispose(bool disposing)
            {
                if (Disposed)
                    return;

                if (disposing)
                {
                    ByteData = null;
                    Metadata = null;
                    Source = null;
                    Width = Height = 0;
                    DpiX = DpiY = 0;
                    NoBytes = NoBands = 0;
                    PixelFormat = PixelFormats.Default;
                    GC.Collect();
                }
                Disposed = true;
            }

            ~LOCImage()
            {
                Dispose(false);
            }

   
        }

        public struct Interpolation
        {


            public static int Bilinear(LOCImage Image, float x, float y, int k)
            {
                int A = 0, B = 0, C = 0, D = 0, Row = 0, Column = 0, Value = 0, Index1 = 0, Index2 = 0, Index3 = 0, Index4 = 0;
                float dx, dy;

                if (x < 0 || y < 0 || x >= Image.Width - 1 || y >= Image.Height - 1)
                {
                    return Value;
                }

                Row = (int)x;
                Column = (int)y;
                dx = x - Row;
                dy = y - Column;


                Index1 = (Column * Image.Width + Row) * Image.ByteBands;
                Index2 = Index1 + Image.ByteBands;
                Index3 = Index1 + Image.Width * Image.ByteBands;
                Index4 = Index1 + Image.Width * Image.ByteBands + Image.ByteBands;

                if (Image.NoBytes == 1)
                {
                    A = Image.ByteData[Index1 + k];
                    B = Image.ByteData[Index2 + k] - A;
                    C = Image.ByteData[Index3 + k] - A;
                    D = -A - B - C + Image.ByteData[Index4 + k];

                    Value = (int)Math.Round(A + B * dx + C * dy + D * dx * dy);
                    Value = (Value >= 255) ? 255 : Value;
                    Value = (Value <= 0) ? 0 : Value;

                }
                else if (Image.NoBytes == 2)
                {
                    A = Image.ByteData[Index1 + k * Image.NoBytes] + Image.ByteData[Index1 + k * Image.NoBytes + 1] * 256;
                    B = Image.ByteData[Index2 + k * Image.NoBytes] + Image.ByteData[Index2 + k * Image.NoBytes + 1] * 256 - A;
                    C = Image.ByteData[Index3 + k * Image.NoBytes] + Image.ByteData[Index3 + k * Image.NoBytes + 1] * 256 - A;
                    D = -A - B - C + Image.ByteData[Index4 + k * Image.NoBytes] + Image.ByteData[Index4 + k * Image.NoBytes + 1] * 256;

                    Value = (int)(A + B * dx + C * dy + D * dx * dy);
                    Value = (Value > 65535) ? 65535 : Value;
                    Value = (Value < 0) ? 0 : Value;

                }
                return Value;
            }

            
        }

    }
}
