using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using System.Linq;
using System.Drawing;
using System.IO;
using System.Text;

using LOC.Image;
using LOC.Photogrammetry;
using NSURF;


namespace LOC
{
    namespace FeatureMatching
    {
        public partial class ImageMatching
        {
            public partial class SURF
            {        
                public float[,] IntetestPoints(LOCImage Image, int Keys, System.Windows.Int32Rect Rect)
                {                   
                    switch (Scale)
                    {
                        case SURF_Scale.Scale1:
                            Filter = new int[1] { 9 };
                            break;
                        case SURF_Scale.Scale2:
                            Filter = new int[1] { 15 };
                            break;
                        case SURF_Scale.Scale3:
                            Filter = new int[1] { 21 };
                            break;
                        case SURF_Scale.Scale4:
                            Filter = new int[1] { 27 };
                            break;
                        case SURF_Scale.Scale5:
                            Filter = new int[1] { 33 };
                            break;
                        case SURF_Scale.MultiScale1:
                            Filter = new int[2] { 9, 15 };
                            break;
                        case SURF_Scale.MultiScale2:
                            Filter = new int[3] { 9, 15, 21 };
                            break;
                        case SURF_Scale.MultiScale3:
                            Filter = new int[4] { 9, 15, 21, 27 };
                            break;
                    }
                    IntegralImage IntegralImage;
                    if (Rect.IsEmpty == true)
                    {
                        IntegralImage = IntegralImage.FromImage(Image);
                    }
                    else
                    {
                        IntegralImage = IntegralImage.FromImage(Image, Rect);
                    }
                    KeyPoint[] KeyPointList = FastHessian.GetKeyPoints(IntegralImage, Keys, Filter);                
                    return SurfDescriptor.DecribePoints(KeyPointList, IntegralImage, IsUpRight, IsExtend);                    

                }


                public void FeatureMatching(LOCImage RefImage, LOCImage TarImage, System.Windows.Int32Rect RefRect, System.Windows.Int32Rect TarRect)
                {

                    RefKPs = IntetestPoints(RefImage, NofRefKeys, RefRect);
                    TarKPs = IntetestPoints(TarImage, NofTarKeys, TarRect);


                    Matching();

                }

            

                private void Matching()
                {
                    if (RefKPs.GetLength(0) < 2 || TarKPs.GetLength(0) < 2)
                        return;

                    KDTree KDTree = KDTree.CreateKDTree(TarKPs);

                    Parallel.For(0, RefKPs.GetLength(0), i =>
                    {
                        float[] KP = new float[66];
                        for (int j = 0; j < 66; j++)
                        {
                            KP[j] = RefKPs[i, j];
                        }
                        float[] KPN = KDTree.BestKP(KP, 20, Thres);
                        if (KPN != null)
                        {
                            InitialMatchPairs.AddOrUpdate(new ImagePoints.Coordinate(RefKPs[i, 64], RefKPs[i, 65], CoordinateFormat.Pixel), new ImagePoints.Coordinate(KPN[64], KPN[65], CoordinateFormat.Pixel), (Key, Value) => new ImagePoints.Coordinate(KPN[64], KPN[65], CoordinateFormat.Pixel));
                        }
                    });
                }


    
               
                public void GetMatches(InOrietation RefIO, InOrietation TarIO, bool IsRansac)
                {
                    this.RefIO = RefIO;
                    this.TarIO = TarIO;
                    CleanMatchPairs.Clear();
                    RansacMatchPairs.Clear();

                    foreach (KeyValuePair<ImagePoints.Coordinate, ImagePoints.Coordinate> Matches in InitialMatchPairs.OrderBy(Ref => Ref.Key.X))
                    {
                        CleanMatchPairs.Add(Matches.Key, Matches.Value);
                    }

                    var Dumpulicate = CleanMatchPairs.GroupBy(Match => Match.Value.X).Where(Match => Match.Count() > 1).SelectMany(Match => Match).ToList();

                    foreach (KeyValuePair<ImagePoints.Coordinate, ImagePoints.Coordinate> Matches in Dumpulicate)
                    {
                        CleanMatchPairs.Remove(Matches.Key);
                    }
                    float Threshold = 10;
                    //RANSAC//
                    foreach (KeyValuePair<ImagePoints.Coordinate, ImagePoints.Coordinate> Matches in CleanMatchPairs)
                    {
                        if (RefIO != null)
                        {
                            Threshold = 5 * RefIO.PixelSize;
                            Matches.Key.FormatTransform(RefIO, CoordinateFormat.mm);
                            Matches.Value.FormatTransform(TarIO, CoordinateFormat.mm);
                        }
                        KeyPoint Ref = new KeyPoint
                        {
                            X = Matches.Key.X,
                            Y = Matches.Key.Y
                        };
                        KeyPoint Tar = new KeyPoint
                        {
                            X = Matches.Value.X,
                            Y = Matches.Value.Y
                        };

                        MatchpointAL.Add(new Match(Ref, Tar));

                        if (IsRansac == false)
                        {
                            RefMatchPTs.Add(new ImagePoints.Coordinate(Ref.X, Ref.Y, CoordinateFormat.mm));
                            TarMatchPTs.Add(new ImagePoints.Coordinate(Tar.X, Tar.Y, CoordinateFormat.mm));
                        }
                    }

                    if (IsRansac == true)
                    {                       
                        Matchmodel = MatchDriver.FilterMatchSet(MatchpointAL, Threshold);
                        if (Matchmodel == null)
                        { return; }

                        for (int i = 0; i < Matchmodel.FittingGround.Count; i++)
                        {
                            Match Pt = Matchmodel.FittingGround[i];
                            RefMatchPTs.Add(new ImagePoints.Coordinate(Pt.Kp1.X, Pt.Kp1.Y, CoordinateFormat.mm));
                            TarMatchPTs.Add(new ImagePoints.Coordinate(Pt.Kp2.X, Pt.Kp2.Y, CoordinateFormat.mm));
                            RansacMatchPairs.Add(new ImagePoints.Coordinate(Pt.Kp1.X, Pt.Kp1.Y, CoordinateFormat.mm), new ImagePoints.Coordinate(Pt.Kp2.X, Pt.Kp2.Y, CoordinateFormat.mm));
                        }
                    }

                }

                public void SaveMatches(string FileName)
                {
                    FileStream SaveFileStream = new FileStream(FileName, FileMode.Create);
                    StreamWriter WriteStream = new StreamWriter(SaveFileStream, Encoding.Default);

                    foreach (KeyValuePair<ImagePoints.Coordinate, ImagePoints.Coordinate> Matches in CleanMatchPairs)
                    {
                        ImagePoints.Coordinate RefPt = new ImagePoints.Coordinate(Matches.Key.X, Matches.Key.Y, CoordinateFormat.mm);
                        ImagePoints.Coordinate TarPt = new ImagePoints.Coordinate(Matches.Value.X, Matches.Value.Y, CoordinateFormat.mm);
                        RefPt.FormatTransform(RefIO, CoordinateFormat.Pixel);
                        TarPt.FormatTransform(TarIO, CoordinateFormat.Pixel);

                        WriteStream.WriteLine(RefPt.X.ToString("0.0000") + "  " + RefPt.Y.ToString("0.0000") + " " + TarPt.X.ToString("0.0000") + "  " + TarPt.Y.ToString("0.0000"));
                    }

                    WriteStream.Close();
                    SaveFileStream.Close();
                }


                public void PaintSURF(string Path, Bitmap img1, Bitmap img2, Color Color, int size, int step)
                {
                    Bitmap mybmp = new Bitmap(img1.Width + img2.Width, img1.Height);
                    Graphics g = Graphics.FromImage(mybmp);

                    g.DrawImage(img1, 0, 0, img1.Width, img1.Height);
                    g.DrawImage(img2, img1.Width, 0, img2.Width, img2.Height);
                    List<ImagePoints.Coordinate> RefPoints = CleanMatchPairs.Keys.ToList();
                    List<ImagePoints.Coordinate> TarPoints = CleanMatchPairs.Values.ToList();
                    int Step1 =1;
                    int Step2 =10;
                    for (int i = 0; i < RefMatchPTs.Count; i = i + Step1 )
                    {
                        RefMatchPTs[i].FormatTransform(RefIO, CoordinateFormat.Pixel);
                        TarMatchPTs[i].FormatTransform(TarIO, CoordinateFormat.Pixel);
                        Point pt = new Point(Convert.ToInt32(RefMatchPTs[i].X), Convert.ToInt32(RefMatchPTs[i].Y));
                        Point ptR = new Point(Convert.ToInt32(TarMatchPTs[i].X), Convert.ToInt32(TarMatchPTs[i].Y));

                        g.DrawLine(new Pen(Color, size), new Point(pt.X, pt.Y), new Point(img1.Width + ptR.X, ptR.Y));
                    }

                    for (int i = 0; i < CleanMatchPairs.Count; i = i + Step2)
                    {
                        if (float.IsInfinity(TarPoints[i].X) != true)
                        {
                            RefPoints[i].FormatTransform(RefIO, CoordinateFormat.Pixel);
                            TarPoints[i].FormatTransform(TarIO, CoordinateFormat.Pixel);
                            Point pt = new Point(Convert.ToInt32(RefPoints[i].X), Convert.ToInt32(RefPoints[i].Y));
                            Point ptR = new Point(Convert.ToInt32(TarPoints[i].X), Convert.ToInt32(TarPoints[i].Y));
                            if (pt.X > 0 && pt.X < img1.Width && ptR.X > 0 && ptR.X < img1.Width)
                            {

                                if (RefMatchPTs.Contains(RefPoints[i]) == false && TarMatchPTs.Contains(TarPoints[i]) == false)
                                {
                                    g.DrawLine(new Pen(Color.Red, size), new Point(pt.X, pt.Y), new Point(img1.Width + ptR.X, ptR.Y));
                                }
                            }
                        }
                    }

                    mybmp.Save(Path, System.Drawing.Imaging.ImageFormat.Jpeg);
                }
            }

        }
    }
}



          