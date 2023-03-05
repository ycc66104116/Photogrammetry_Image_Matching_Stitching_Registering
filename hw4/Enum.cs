using System.ComponentModel;
namespace LOC
{
    public enum CoordinateFormat { mm, Pixel }
  
    public enum ImageFormat { Jpeg, Tiff, Bmp, Png, Wmp, Gif }

    public enum SURF_Scale
    {
        [Description("Filter = 9")]
        Scale1,

        [Description("Filter = 15")]
        Scale2,

        [Description("Filter = 21")]
        Scale3,

        [Description("Filter = 27")]
        Scale4,

        [Description("Filter = 33")]
        Scale5,

        [Description("Filter = 9+15")]
        MultiScale1,

        [Description("Filter = 9+15+21")]
        MultiScale2,

        [Description("Filter = 9+15+21+27")]
        MultiScale3 
    }


}
