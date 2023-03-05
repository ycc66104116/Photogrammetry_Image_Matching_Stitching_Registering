using System;
using System.Threading.Tasks;
using System.Linq;
using System.Collections.Generic;
using System.Diagnostics;


namespace LOC
{
    namespace Photogrammetry
    {
        public class LensDistortion
        {

            public int OffsetX;
            public int OffsetY;
           
            private InOrietation UndistortedIO;

            /// <summary>
            /// 輸入具透鏡畸變的原始座標，由相機內方位參數改正回傳無透鏡畸變的座標
            /// </summary>
            /// <param name="OriginalCoordinate">具透鏡畸變的原始座標</param>
            /// <param name="IO">相機內方位參數</param>
            /// <returns>無透鏡畸變的座標</returns>
            public ImagePoints.Coordinate RemoveLensDistortion(ImagePoints.Coordinate OriginalCoordinate, InOrietation IO)
            {
                float r2, r4, r6, r8, r10, dx2, dy2, dx, dy, dxy, RadialEquation, Radial_x, Radial_y, Decenter_x, Decenter_y;
                ImagePoints.Coordinate CorrectCoordinate = new ImagePoints.Coordinate();
                OriginalCoordinate.FormatTransform(IO, CoordinateFormat.mm);

                dx = OriginalCoordinate.X - IO.Xp;
                dy = OriginalCoordinate.Y - IO.Yp;
                dx2 = dx * dx; dy2 = dy * dy; dxy = dx * dy;
                r2 = (dx2 + dy2); r4 = r2 * r2; r6 = r2 * r4; r8 = r2 * r6; r10 = r2 * r8;
                RadialEquation = IO.K1 * r2 + IO.K2 * r4 + IO.K3 * r6 + IO.K4 * r8 + IO.K5 * r10;

                Radial_x = RadialEquation * dx;
                Radial_y = RadialEquation * dy;
                Decenter_x = IO.P1 * (r2 + 2 * dx2) + 2 * IO.P2 * dxy;
                Decenter_y = IO.P2 * (r2 + 2 * dy2) + 2 * IO.P1 * dxy;

                CorrectCoordinate.X = dx + Radial_x + Decenter_x;
                CorrectCoordinate.Y = dy + Radial_y + Decenter_y;

                return CorrectCoordinate;
            }

            /// <summary>
            /// 輸入無透鏡畸變的座標，由內方位參數反算具透鏡畸變的座標
            /// </summary>
            /// <param name="CorrectCoordinate">無透鏡畸變之座標</param>
            /// <param name="IO">相機內方位參數</param>
            /// <returns>具透鏡畸變的座標</returns>
            public ImagePoints.Coordinate AddLensDistortion(ImagePoints.Coordinate CorrectCoordinate, InOrietation IO)
            {
                float r2, r4, r6, r8, r10, dx2, dy2, dx, dy, dxy, Radial_x, Radial_y, Decenter_x, Decenter_y, Temptx = 0, Tempty = 0, Pixel, RadialEquation;
                ImagePoints.Coordinate OriginalCoordinate = new ImagePoints.Coordinate();
                CorrectCoordinate.FormatTransform(IO, CoordinateFormat.mm);

                OriginalCoordinate.X = CorrectCoordinate.X;
                OriginalCoordinate.Y = CorrectCoordinate.Y;
                Pixel = IO.PixelSize * IO.PixelSize / 10000;

                while ((OriginalCoordinate.X - Temptx) * (OriginalCoordinate.X - Temptx) + (OriginalCoordinate.Y - Tempty) * (OriginalCoordinate.Y - Tempty) > Pixel)
                {
                    Temptx = OriginalCoordinate.X;
                    Tempty = OriginalCoordinate.Y;
                    dx = OriginalCoordinate.X - IO.Xp;
                    dy = OriginalCoordinate.Y - IO.Yp;
                    dx2 = dx * dx; dy2 = dy * dy; dxy = dx * dy;
                    r2 = (dx2 + dy2); r4 = r2 * r2; r6 = r2 * r4; r8 = r2 * r6; r10 = r2 * r8;

                    RadialEquation = (IO.K1 * r2 + IO.K2 * r4 + IO.K3 * r6 + IO.K4 * r8 + IO.K5 * r10);

                    Radial_x = RadialEquation * dx;
                    Radial_y = RadialEquation * dy;
                    Decenter_x = IO.P1 * (r2 + 2 * dx2) + 2 * IO.P2 * dxy;
                    Decenter_y = IO.P2 * (r2 + 2 * dy2) + 2 * IO.P1 * dxy;

                    OriginalCoordinate.X = CorrectCoordinate.X + IO.Xp - Radial_x - Decenter_x;
                    OriginalCoordinate.Y = CorrectCoordinate.Y + IO.Yp - Radial_y - Decenter_y;
                }

                return OriginalCoordinate;
            }


       
        }
    }

}
