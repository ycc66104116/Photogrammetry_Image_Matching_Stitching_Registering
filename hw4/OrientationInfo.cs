using System;
using System.Collections.Generic;


namespace LOC
{
    namespace Photogrammetry
    {
        public class InOrietation
        {
            private string cameraNumber, cameraName, iD;
            private int imageWidth, imageHeight;
            private int undistortedW, undistortedH;
            private float sensorWidth, sensorHeight;
            private float focalLength, pixelSize, adjustedFocalLength, xp, yp, k1, k2, k3, k4, k5, p1, p2, b1, b2;
            private List<ImagePoints.Coordinate> borderMidPoints;
            private float halfHeight, halfWidth;
            public int Resolution;
            public string CameraNumber { get => cameraNumber; set => cameraNumber = value; }
            public string CameraName { get => cameraName; set => cameraName = value; }
            public string ID { get => iD; set => iD = value; }
            public int ImageWidth { get => imageWidth; set => imageWidth = value; }
            public int ImageHeight { get => imageHeight; set => imageHeight = value; }
            public float SensorWidth { get => sensorWidth; set => sensorWidth = value; }
            public float SensorHeight { get => sensorHeight; set => sensorHeight = value; }
            public float FocalLength { get => focalLength; set => focalLength = value; }
            public float AdjustedFocalLength { get => adjustedFocalLength; set => adjustedFocalLength = value; }

            public float PixelSize { get => pixelSize; set => pixelSize = value; }
            public float Xp { get => xp; set => xp = value; }
            public float Yp { get => yp; set => yp = value; }
            public float K1 { get => k1; set => k1 = value; }
            public float K2 { get => k2; set => k2 = value; }
            public float K3 { get => k3; set => k3 = value; }
            public float K4 { get => k4; set => k4 = value; }
            public float K5 { get => k5; set => k5 = value; }
            public float P1 { get => p1; set => p1 = value; }
            public float P2 { get => p2; set => p2 = value; }
            public float B1 { get => b1; set => b1 = value; }
            public float B2 { get => b2; set => b2 = value; }
            public float PixelScale { get; set; }
            public List<ImagePoints.Coordinate> BorderMidPoints { get => borderMidPoints; set => borderMidPoints = value; }
            public int UndistortedW { get => undistortedW; set => undistortedW = value; }
            public int UndistortedH { get => undistortedH; set => undistortedH = value; }
            public float HalfWidth { get => halfWidth; set => halfWidth = value; }
            public float HalfHeight { get => halfHeight; set => halfHeight = value; }

            public void GetSensorSize()
            {
                SensorWidth = (float)Math.Round(ImageWidth * PixelSize, 10);
                SensorHeight = (float)Math.Round(ImageHeight * PixelSize, 10);
                HalfWidth = ImageWidth / 2;
                HalfHeight = ImageHeight / 2;
                Resolution = ImageWidth * ImageHeight;
                AdjustedFocalLength = (float)Math.Round(FocalLength, 1);
                PixelScale = 1 / PixelSize;
                BorderMidPoints = new List<ImagePoints.Coordinate>
                {
                    new ImagePoints.Coordinate(0,0,CoordinateFormat.Pixel),
                    new ImagePoints.Coordinate(ImageWidth/2, 0, CoordinateFormat.Pixel),
                    new ImagePoints.Coordinate(ImageWidth,0,CoordinateFormat.Pixel),
                    new ImagePoints.Coordinate(ImageWidth, ImageHeight/2, CoordinateFormat.Pixel),
                    new ImagePoints.Coordinate(ImageWidth, ImageHeight, CoordinateFormat.Pixel),
                    new ImagePoints.Coordinate(ImageWidth/2, ImageHeight, CoordinateFormat.Pixel),
                    new ImagePoints.Coordinate(0, ImageHeight, CoordinateFormat.Pixel),
                    new ImagePoints.Coordinate(0, ImageHeight/2, CoordinateFormat.Pixel),
                };

            }


            public InOrietation()
            {

            }
            public InOrietation(int ImageWidth, int ImageHeight, float FocalLength, float PixelSize)
            {
                this.ImageWidth = ImageWidth;
                this.ImageHeight = ImageHeight;
                this.FocalLength = FocalLength;
                this.PixelSize = PixelSize;
                PixelScale = 1 / pixelSize;
                GetSensorSize();
            }
            public InOrietation(int ImageWidth, int ImageHeight, float FocalLength, float PixelSize, float xp, float yp, float k1, float k2, float k3, float k4, float k5, float p1, float p2)
            {
                this.ImageWidth = ImageWidth;
                this.ImageHeight = ImageHeight;
                this.FocalLength = FocalLength;
                this.PixelSize = PixelSize;
                this.xp = xp;
                this.yp = yp;
                this.k1 = k1;
                this.k2 = k2;
                this.k3 = k3;
                this.k4 = k4;
                this.k5 = k5;
                this.p1 = p1;
                this.p2 = p2;


                PixelScale = 1 / pixelSize;
                GetSensorSize();
            }

        }

        public class ImagePoints
        {
            public class Coordinate
            {
                private float x, y, vx, vy;
                private CoordinateFormat format;
                public bool IsDistortion;

                public float X { get => x; set => x = value; }
                public float Y { get => y; set => y = value; }

                public CoordinateFormat Format { get => format; set => format = value; }

                public Coordinate()
                {

                }
                public Coordinate(float X, float Y, CoordinateFormat Format)
                {
                    this.X = X;
                    this.Y = Y;
                    this.Format = Format;
                    IsDistortion = true;
                }


                public void FormatTransform(InOrietation IO, CoordinateFormat Pixel)
                {
                    // ImageCoordinate TransformImageCoordinate = new ImageCoordinate();
                    switch (Pixel)
                    {
                        //像元座標系統至像座標系統
                        case CoordinateFormat.mm:
                            if (Format == CoordinateFormat.mm)
                            {
                                X = X;
                                Y = Y;
                                break;
                            }
                            else
                            {
                                X = (X - IO.HalfWidth) * IO.PixelSize;
                                Y = (IO.HalfHeight - Y) * IO.PixelSize;
                                Format = CoordinateFormat.mm;
                                break;
                            }
                        //像座標系統至像元座標系統
                        case CoordinateFormat.Pixel:
                            if (Format == CoordinateFormat.Pixel)
                            {
                                X = X;
                                Y = Y;
                                break;
                            }
                            else
                            {
                                X = (X * IO.PixelScale + IO.HalfWidth);
                                Y = (-Y * IO.PixelScale + IO.HalfHeight);
                                Format = CoordinateFormat.Pixel;
                                break;
                            }
                    }
                }

                public static Coordinate operator *(Coordinate ImageCoordinate, float Scale)
                {
                    ImageCoordinate.X = ImageCoordinate.X * Scale;
                    ImageCoordinate.Y = ImageCoordinate.Y * Scale;
                    return ImageCoordinate;
                }
                public static Coordinate operator /(Coordinate ImageCoordinate, float Scale)
                {
                    ImageCoordinate.X = ImageCoordinate.X / Scale;
                    ImageCoordinate.Y = ImageCoordinate.Y / Scale;
                    return ImageCoordinate;
                }
                public static Coordinate operator +(Coordinate ImageCoordinate, float Shift)
                {
                    ImageCoordinate.X = ImageCoordinate.X + Shift;
                    ImageCoordinate.Y = ImageCoordinate.Y + Shift;
                    return ImageCoordinate;
                }
            }
        }
    }
}

