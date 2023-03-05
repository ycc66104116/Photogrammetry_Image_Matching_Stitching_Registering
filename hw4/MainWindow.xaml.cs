using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.IO;
using System.Drawing;
using LOC.Image;
using Microsoft.Win32;
using LOC;
using LOC.FeatureMatching;
using LOC.Photogrammetry;
using LOC.Photogrammetry.CoordinateTransform;


namespace hw4
{
    /// <summary>
    /// MainWindow.xaml 的互動邏輯
    /// </summary>
    public partial class MainWindow : Window
    {
        public MainWindow()
        {
            InitializeComponent();
        }
        LOCImage RImage = null;
        LOCImage TImage = null;
        String referenceFilename;
        String targetFilename;
        String matchingFilename = "D:\\四下\\近景攝影測量\\近景hw4\\output_0528\\Drawing.jpg";
        String stitchingFilename = "D:\\四下\\近景攝影測量\\近景hw4\\output_0528\\stitching.jpg";
        String RigisteringFilename = "D:\\四下\\近景攝影測量\\近景hw4\\output_0528\\rigistering.jpg";
        String referencecompareFilename= "D:\\四下\\近景攝影測量\\近景hw4\\output_0528\\referencecompare.jpg";
        String targetcompareFilename= "D:\\四下\\近景攝影測量\\近景hw4\\output_0528\\targetcompare.jpg";
        ProjectiveTransform BackwardPT;
        ProjectiveTransform ForwardPT;
        private void Matching1_Click(object sender, RoutedEventArgs e)
        {
            ImageMatching.SURF SURF = new ImageMatching.SURF(12000, 12000, (SURF_Scale)0, true, false, 0.8f);
            SURF.FeatureMatching(RImage, TImage, Int32Rect.Empty, Int32Rect.Empty);
            InOrietation IO = new InOrietation(RImage.Width, RImage.Height, 1f, 1f);

            
            SURF.GetMatches(IO, IO, true);
            SURF.SaveMatches("D:\\四下\\近景攝影測量\\近景hw4\\output_0528\\Matching.txt");
            BackwardPT = new ProjectiveTransform(2.5f);
            ForwardPT = new ProjectiveTransform(2.5f);
            BackwardPT.Adjustment(SURF.RefMatchPTs, SURF.TarMatchPTs, new LeastSquare());
            ForwardPT.Adjustment(SURF.TarMatchPTs, SURF.RefMatchPTs, new LeastSquare());
            SURF.PaintSURF(matchingFilename, new Bitmap(referenceFilename), new Bitmap(targetFilename), System.Drawing.Color.Green, 5, 1);

            using(var stream=new FileStream(matchingFilename,FileMode.Open,FileAccess.Read,FileShare.Read))
            {
                matching.Source = BitmapFrame.Create(stream, BitmapCreateOptions.None, BitmapCacheOption.OnLoad);

                
            }
        
        }

        private void Openr_Click(object sender, RoutedEventArgs e)
        {
            OpenFileDialog OFD = new OpenFileDialog();
            OFD.ShowDialog();
            referenceFilename = OFD.FileName;
            reference.Source = new BitmapImage(new Uri(referenceFilename));
            RImage = new LOCImage(referenceFilename, Int32Rect.Empty);
            
        }

        private void Opent_Click(object sender, RoutedEventArgs e)
        {
            OpenFileDialog OFD = new OpenFileDialog();
            OFD.ShowDialog();
            targetFilename = OFD.FileName;
            target.Source = new BitmapImage(new Uri(targetFilename));
            TImage = new LOCImage(targetFilename, Int32Rect.Empty);
            
        }
        //影像拼接
        private void Stitching_Click(object sender, RoutedEventArgs e)
        {
            List<ImagePoints.Coordinate> Corner = new List<ImagePoints.Coordinate>
          {
              new ImagePoints.Coordinate(0,0,CoordinateFormat.Pixel),
              new ImagePoints.Coordinate(TImage.Width,0,CoordinateFormat.Pixel),
              new ImagePoints.Coordinate(0,TImage.Height,CoordinateFormat.Pixel),
              new ImagePoints.Coordinate(TImage.Width,TImage.Height,CoordinateFormat.Pixel)
          };
            InOrietation IO = new InOrietation(TImage.Width, TImage.Height, 1f, 1f);
            for(int i=0;i<4;i++)
            {
                //格式轉換
                Corner[i].FormatTransform(IO, CoordinateFormat.mm);
            }
            //目標影像角點正轉換至參考影像坐標系
            Corner = ForwardPT.Transform(Corner);
            //角點列表加入參考影像角點
            Corner.Add(new ImagePoints.Coordinate(0, 0, CoordinateFormat.Pixel));
            Corner.Add(new ImagePoints.Coordinate(RImage.Width, 0, CoordinateFormat.Pixel));
            Corner.Add(new ImagePoints.Coordinate(0, RImage.Height, CoordinateFormat.Pixel));
            Corner.Add(new ImagePoints.Coordinate(RImage.Width, RImage.Height, CoordinateFormat.Pixel));
            for(int i=4;i<8;i++)
            {
                //全部角點格式轉換
                Corner[i].FormatTransform(IO, CoordinateFormat.mm);
            }
            //找出參考影像 加上 轉換至參考影像坐標系之目標影像 的 最大最小xy 範圍
            float Max_x = -999999f, Max_y = -999999f, Min_x = 999999f, Min_y = 999999f;
            for(int i=0;i<8;i++)
            {
                if(Corner[i].X>Max_x)
                {
                    Max_x = Corner[i].X;
                }
                if (Corner[i].Y > Max_y)
                {
                    Max_y = Corner[i].Y;
                }
                if (Corner[i].X <Min_x)
                {
                    Min_x = Corner[i].X;
                }
                if (Corner[i].Y < Min_y)
                {
                    Min_y = Corner[i].Y;
                }            
            }
            //計算拼接影像大小
            int StitchWidth = (int)(Max_x - Min_x);
            int StitchHeight = (int)(Max_y - Min_y);
            //計算拼接影像中心點與參考影像中心點位移量
            int OffsetWidth = (int)(Max_x + Min_x)/2;
            int OffsetHeight = (int)(Max_y +Min_y)/2;
            byte[] Imagebyte = new byte[StitchWidth * StitchHeight * RImage.NoBands];
            //以計算之拼接影像大小宣告拼接影像
            LOCImage StitchImage = new LOCImage(StitchWidth, StitchHeight, 96, 96, RImage.PixelFormat, null);
            int Width = RImage.Width, Height = RImage.Height, Bands = RImage.NoBands;
            for(int i=0;i< StitchWidth;i++)
            {
                int Index = 0;
                float RefX = 0,RefY=0,TarX = 0,TarY=0,StitchX=0,StitchY = 0;
                for (int j = 0; j < StitchHeight; j++)
                {
                    Index = (j * StitchWidth + i) * Bands;
                    //修正影像中心點位置
                    StitchX = (i - StitchWidth / 2) + OffsetWidth;
                    StitchY = (StitchHeight / 2 - j) + OffsetHeight;
                    RefX = StitchX + Width / 2;
                    RefY = -StitchY + Height / 2;
                    for (int k = 0; k < Bands; k++)
                    {  
                        //內插在拼接影像中的參考影像(僅有位移量差異)
                        Imagebyte[Index + k] = (byte)Interpolation.Bilinear(RImage, RefX, RefY, k);                        
                    }
                    //逆轉換拼接影像至目標影像
                    BackwardPT.Transform(StitchX, StitchY);
                    TarX = (BackwardPT.TransformPt[0] + Width / 2);
                    TarY = (-BackwardPT.TransformPt[1] + Height / 2);                 
                    for (int k=0;k<Bands;k++)
                    {
                        if (Imagebyte[Index+k]==0)
                        {
                            //內插在拼接影像中的目標影像(找尋與拼接影像各像素對應的值)
                            Imagebyte[Index + k] = (byte)Interpolation.Bilinear(TImage, TarX, TarY, k);
                        }
                    }
                }
            }
            StitchImage.ByteData = Imagebyte;
            StitchImage.Save(stitchingFilename, ImageFormat.Jpeg);
            using (var stream = new FileStream(stitchingFilename, FileMode.Open, FileAccess.Read, FileShare.Read))
            {
                matching.Source = BitmapFrame.Create(stream, BitmapCreateOptions.None, BitmapCacheOption.OnLoad);
            }
        }

        private void Register_Click(object sender, RoutedEventArgs e)
        {
            //另存原始影像以便比較
            RImage.Save(referencecompareFilename, ImageFormat.Jpeg);
            TImage.Save(targetcompareFilename, ImageFormat.Jpeg);

            List<ImagePoints.Coordinate> Corner = new List<ImagePoints.Coordinate>
          {
              new ImagePoints.Coordinate(0,0,CoordinateFormat.Pixel),
              new ImagePoints.Coordinate(TImage.Width,0,CoordinateFormat.Pixel),
              new ImagePoints.Coordinate(0,TImage.Height,CoordinateFormat.Pixel),
              new ImagePoints.Coordinate(TImage.Width,TImage.Height,CoordinateFormat.Pixel)
          };
            InOrietation IO = new InOrietation(TImage.Width, TImage.Height, 1f, 1f);
            //參考影像與目標影像大小相同
            byte[] Imagebyte = new byte[RImage.Width * RImage.Height * RImage.NoBands];
            //Rigistered 影像與原本相同
            LOCImage RigisterImage = new LOCImage(RImage.Width, RImage.Height, 96, 96, RImage.PixelFormat, null);
            for (int i = 0; i < RImage.Width; i++)
            {
                int Index = 0;
                float TarX = 0, TarY = 0, RigisterX = 0, RigisterY = 0;
                for (int j = 0; j < RImage.Height; j++)
                {
                    Index = (j * RImage.Width + i) * RImage.NoBands;
                    //Rigistered 影像大小與原本相同
                    RigisterX = (i - RImage.Width / 2) ;
                    RigisterY = (RImage.Height / 2 - j) ;
                    //以Rigistered 影像(與參考影像相同坐標系)逆轉換至目標影像進行內插獲取對應值
                    BackwardPT.Transform(RigisterX, RigisterY);
                    TarX = (BackwardPT.TransformPt[0] + RImage.Width / 2);
                    TarY = (-BackwardPT.TransformPt[1] + RImage.Height / 2);
                    for (int k = 0; k < RImage.NoBands; k++)
                    {
                        if (Imagebyte[Index + k] == 0)
                        {
                            Imagebyte[Index + k] = (byte)Interpolation.Bilinear(TImage, TarX, TarY, k);
                        }
                    }
                }
            }
            RigisterImage.ByteData = Imagebyte;
            RigisterImage.Save(RigisteringFilename, ImageFormat.Jpeg);
            using (var stream = new FileStream(RigisteringFilename, FileMode.Open, FileAccess.Read, FileShare.Read))
            {
                matching.Source = BitmapFrame.Create(stream, BitmapCreateOptions.None, BitmapCacheOption.OnLoad);
            }
        }
    }
}
