//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

using System.Windows.Threading;
using Microsoft.Research.DynamicDataDisplay;
using Microsoft.Research.DynamicDataDisplay.DataSources;
namespace Microsoft.Samples.Kinect.ColorBasics
{
    using System;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;

    using System.Collections.Generic;
    using System.Windows.Input;

    using System.Windows.Media.Media3D;


    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        #region Initialize

        #region plotter
        private ObservableDataSource<System.Windows.Point> dataSource1 = new ObservableDataSource<System.Windows.Point>();
        private ObservableDataSource<System.Windows.Point> dataSource2 = new ObservableDataSource<System.Windows.Point>();
        private ObservableDataSource<System.Windows.Point> dataSource3 = new ObservableDataSource<System.Windows.Point>();
        private ObservableDataSource<System.Windows.Point> dataSource4 = new ObservableDataSource<System.Windows.Point>();
        private ObservableDataSource<System.Windows.Point> dataSource5 = new ObservableDataSource<System.Windows.Point>();
        private ObservableDataSource<System.Windows.Point> dataSource6 = new ObservableDataSource<System.Windows.Point>();
        private ObservableDataSource<System.Windows.Point> dataSource7 = new ObservableDataSource<System.Windows.Point>();
        private ObservableDataSource<System.Windows.Point> dataSource8 = new ObservableDataSource<System.Windows.Point>();
        private DispatcherTimer timer = new DispatcherTimer();
        private DispatcherTimer timer2 = new DispatcherTimer();
        private int ploti = 0;
        private int vari_ploti = 0;
        private int slide_framecount = 60;

        private double variable_V;
        private double variable_A;
        private double variable_R;
        private double variable_W;
        #endregion
        public struct Depvariation{

            public double [,]depvalue;
            public int []counter;
            public double[] variation;
            public double[] mean;
            public double[,] V;
            public int[] Environment_label;//1 is floor 2 is can't catch 3 is background 4 is moving
            public int[] ELB_ORG;//1 is floor 2 is can't catch 3 is background 4 is moving
            public int[] object_label;
            public int[] object_num;
            public int totoal_object;
        };

        public struct Fall_rule {

            public int[] fall_down_timer;
            public int[] spee_zero_timer;
            public Vector3D[] now_center;
            public Vector3D[] before_center;
            public double[] acceleration;
            public double[] befor_speed;
            public double[] now_speed;
            public Boolean[] speed_trig;
            public int[] top;
            public int[] right;
            public int[] left;
            public int[] buttom;
            public int[] object_timer;
        }
        public struct Feature {
            public double hwratio_height;
            public double hwratio_width;
            public double vertical;
            public double vbefore;
            public int toatal_area;
            public int floor_area;
            public double areabefore;
            public int leftwidth;
            public int rightwidth;
            public double mostwidth;
            public double mwbefore;
            public double low;
            public double leftw;
            public double rightw;

            public int top_l;
            public int low_l;
            public int right_l;
            public int left_l;
            public int fabefore;
        }

        private KinectSensor sensor;

        private WriteableBitmap colorBitmap;
        System.Windows.Point[] object_center = new System.Windows.Point[100];
        System.Windows.Point[] before_center = new System.Windows.Point[100];
        private WriteableBitmap DepthBitmap;
        private WriteableBitmap floorBitmap;
        private WriteableBitmap backdepthBitmap;
        private DepthImagePixel[] depthPixels;
        private DepthImagePixel[] dep4background;
        private ImageBrush Background = new ImageBrush();
        SkeletonPoint[] ColorInSkel;
        SkeletonPoint[] NewCoordinate;
        SkeletonPoint[] ObjectCoordinate;
        ColorImagePoint[] colorCoordinate;
        private SolidColorBrush[] ColorBrush;
        string t1;
        string feature;
        string feature2;
        private byte[] colorPixels;
        private byte[] DepthPixels;
        private byte[] floorPixels;
        private byte[] backPixels;
        private double[,] depvariation;
        int[] label_group = new int[640 * 480];
        int[] label_num = new int[640 * 480];
        Depvariation d = new Depvariation();
        Fall_rule f = new Fall_rule();
        Feature[] focus = new Feature[10];
        private double[] Vari;
        private int[] multi_object_label;
        private int framecount=0,how_many_object,label_count=0;
        private List<int> thisdepthx = new List<int>() { };
        private List<int> thisdepthy = new List<int>() { };
        Random random = new Random();
        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        int MouseX = 320, MouseY = 240,curobject=0,update_num=0,coming_time=0,test=0;
        double Xr, Yr, Zr = -1,RMX = 320, RMY = 240, Right_lengthX = 0, Right_lengthY = 0, Right_lengthZ = 0,angel=0;
        double[] speed= new double[100];
        Boolean transfer_value = false, startlabel = false, floor_label=false,fall_down_lable=false,updating=false;
        Vector3D origin;
        Vector3D []center_cur = new Vector3D[100];
        Vector3D[] center_bef = new Vector3D[100];
        
        List<int> floor_point = new List<int>() { };
       // List<int> object_point = new List<int>() { };
        List<List<int>> object_point = new List<List<int>>();
        List<int> obj2 = new List<int>() { };
        List<int> lower_point = new List<int>() { };
        /// <summary>
        /// Width of output drawing
        /// </summary>

        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;
        public DrawingContext drawtest;
        #endregion
        public MainWindow()
        {
            InitializeComponent();
            ColorBrush = new[] { System.Windows.Media.Brushes.Gray,
                System.Windows.Media.Brushes.PaleVioletRed, 
                System.Windows.Media.Brushes.DeepSkyBlue,
                System.Windows.Media.Brushes.Yellow, 
                System.Windows.Media.Brushes.Green, 
                System.Windows.Media.Brushes.Black, 
                System.Windows.Media.Brushes.White, 
                System.Windows.Media.Brushes.Brown };
            
         
        }
        private void WindowLoaded(object sender, RoutedEventArgs e)
        {
            #region plotter initialize
            plotter1.AddLineGraph(dataSource1, Colors.Green, 2, " ");
            timer.Interval = TimeSpan.FromSeconds(0.5);
            timer.Tick += new EventHandler(AnimatedPlot_center);
            timer.IsEnabled = true;
            plotter1.Viewport.FitToView();

            plotter2.AddLineGraph(dataSource2, Colors.Blue, 2, " ");
            timer.Interval = TimeSpan.FromSeconds(0.5);
            timer.Tick += new EventHandler(AnimatedPlot_hwratio);
            timer.IsEnabled = true;
            plotter2.Viewport.FitToView();

            plotter3.AddLineGraph(dataSource3, Colors.Olive, 2, " ");
            timer.Interval = TimeSpan.FromSeconds(0.5);
            timer.Tick += new EventHandler(AnimatedPlot_widthest);
            timer.IsEnabled = true;
            plotter3.Viewport.FitToView();

            plotter4.AddLineGraph(dataSource4, Colors.Red, 2, " ");
            timer.Interval = TimeSpan.FromSeconds(0.5);
            timer.Tick += new EventHandler(AnimatedPlot_area);
            timer.IsEnabled = true;
            plotter4.Viewport.FitToView();

            plotter1_Copy.AddLineGraph(dataSource5, Colors.Green, 2, " ");
            timer2.Interval = TimeSpan.FromSeconds(2);
            timer2.Tick += new EventHandler(AnimatedPlot_center_Variation);
            timer2.IsEnabled = true;
            plotter1_Copy.Viewport.FitToView();

            plotter2_Copy.AddLineGraph(dataSource6, Colors.Blue, 2, " ");
            timer2.Interval = TimeSpan.FromSeconds(2);
            timer2.Tick += new EventHandler(AnimatedPlot_hwratio_Variation);
            timer2.IsEnabled = true;
            plotter2_Copy.Viewport.FitToView();

            plotter3_Copy.AddLineGraph(dataSource7, Colors.Olive, 2, " ");
            timer2.Interval = TimeSpan.FromSeconds(2);
            timer2.Tick += new EventHandler(AnimatedPlot_widthest_Variation);
            timer2.IsEnabled = true;
            plotter3_Copy.Viewport.FitToView();

            plotter4_Copy.AddLineGraph(dataSource8, Colors.Red, 2, " ");
            timer2.Interval = TimeSpan.FromSeconds(2);
            timer2.Tick += new EventHandler(AnimatedPlot_area_Variation);
            timer2.IsEnabled = true;
            plotter4_Copy.Viewport.FitToView();
            #endregion
            label2.Content = slider.Value.ToString();

            this.drawingGroup = new DrawingGroup();
            this.imageSource = new DrawingImage(this.drawingGroup);
            foreach (var potentialSensor in KinectSensor.KinectSensors)
            {
                if (potentialSensor.Status == KinectStatus.Connected)
                {
                    this.sensor = potentialSensor;
                    break;
                }
            }
            // this.drawingGroup = new DrawingGroup();                         // Create the drawing group we'll use for drawing
            //   this.imageSource = new DrawingImage(this.drawingGroup);         // Create an image source that we can use in our image control
            // Display the drawing using our image control
            #region initialize variable  
            if (null != this.sensor)
            {
                variable_V = 0;
                variable_A = 0;
                variable_R = 0;
                variable_W = 0;
                this.depvariation = new double[10,640*480];
                d.totoal_object = 0;
                d.counter = new int[640 * 480];
                d.object_num = new int[200];
                d.object_label = new int[640 * 480];
                d.depvalue = new double[30,640 * 480];
                d.V = new double[30, 640 * 480];
                d.mean = new double[640 * 480];
                d.variation = new double[640 * 480];
                d.Environment_label = new int[640 * 480];
                d.ELB_ORG = new int[640 * 480];
                this.multi_object_label = new int[640 * 480];
                this.Vari = new double[640 * 480];
                f.fall_down_timer = new int[200];
                f.acceleration = new double[200];
                f.now_center = new Vector3D[200];
                f.now_speed = new double[200];
                f.befor_speed = new double[200];
                f.before_center = new Vector3D[200];
                f.spee_zero_timer = new int[200];
                f.speed_trig = new Boolean[200];
                f.buttom = new int[200];
                f.top = new int[200];
                f.right = new int[200];
                f.left = new int[200];
                f.object_timer = new int[200];
                
                NewCoordinate = new SkeletonPoint[307200];
                d.V.Initialize();
                // Turn on the color stream to receive color frames
                this.sensor.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);
                colorCoordinate = new ColorImagePoint[640 * 480];
                this.colorPixels = new byte[this.sensor.ColorStream.FramePixelDataLength];
                this.colorBitmap = new WriteableBitmap(this.sensor.ColorStream.FrameWidth, this.sensor.ColorStream.FrameHeight, 96.0, 96.0, PixelFormats.Bgr32, null);
                this.Image.Source = this.colorBitmap;
                //Image.Source = this.imageSource;     
              //  Image.Source = this.imageSource;
                this.sensor.ColorFrameReady += this.SensorColorFrameReady;
                
                // depth
                this.sensor.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);             
                this.DepthPixels = new byte[this.sensor.ColorStream.FramePixelDataLength];
                this.backPixels = new byte[this.sensor.ColorStream.FramePixelDataLength];     
                this.depthPixels = new DepthImagePixel[this.sensor.DepthStream.FramePixelDataLength];
                this.dep4background = new DepthImagePixel[this.sensor.DepthStream.FramePixelDataLength];  
                //this.DepthBitmap = new WriteableBitmap(this.sensor.DepthStream.FrameWidth, this.sensor.DepthStream.FrameHeight, 96.0, 96.0, PixelFormats.Bgr32, null);
                this.backdepthBitmap = new WriteableBitmap(this.sensor.DepthStream.FrameWidth, this.sensor.DepthStream.FrameHeight, 96.0, 96.0, PixelFormats.Bgr32, null);
               // this.Image2.Source = this.DepthBitmap;
                
               // this.Image4.Source = this.backdepthBitmap;
                this.Image4.Source = this.imageSource;
                this.floorBitmap = new WriteableBitmap(this.sensor.DepthStream.FrameWidth, this.sensor.DepthStream.FrameHeight, 96.0, 96.0, PixelFormats.Bgr32, null);
                this.floorPixels = this.colorPixels;
            
             //   this.Image3.Source = this.floorBitmap;
               
                this.sensor.DepthFrameReady += this.SensorDepthFrameReady;
                // Turn on the skeleton stream to receive skeleton frames
                this.sensor.SkeletonStream.Enable();

                // Add an event handler to be called whenever there is new color frame data
                this.sensor.SkeletonFrameReady += this.SensorSkeletonFrameReady;
                // Start the sensor!
                try
                {
                    this.sensor.Start();
                }
                catch (IOException)
                {
                    this.sensor = null;
                }
            }
#endregion

        }
        private void WindowClosing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (null != this.sensor)
            {
                this.sensor.Stop();
            }
        }
        private void SensorColorFrameReady(object sender, ColorImageFrameReadyEventArgs e)
        {  
            using (ColorImageFrame colorFrame = e.OpenColorImageFrame())
            {
                if (colorFrame != null)
                {
                    colorFrame.CopyPixelDataTo(this.colorPixels);
                    

                    // Write the pixel data into our bitmap

                    ColorInSkel = ColorToSkel(640, 480);
                    this.colorBitmap.WritePixels(
                           new Int32Rect(0, 0, 640, 480),
                           this.colorPixels,
                           this.colorBitmap.PixelWidth * sizeof(int),
                           0);
                }             
            }           
        }
        #region euler tranfer
        /*************
         * Euler    transfer the coordinate
         * X rotate {1,0,0
         *           0,Cos(theta),-Sin(theta)
         *           0,Sin(theta),Cos(theta)}
         * Y rotate {Cos(theta),0,Sin(theta)
         *           0,1,0
         *           -Sin(theta),0,Cos(theta)}
         * Z rotate {Cos(theta),-Sin(theta),0        
         *           0,Sin(theta),Cos(theta)
         *           0,0,1}
         * *************/
        private Vector3D coordinate_rotate(double theta)
        {
            double degree = Math.PI/180,x,y,z;
            
            Vector3D real_world_point = new Vector3D();
            Vector3D floor = new Vector3D();
            Matrix3D Euler          = new Matrix3D(1, 0, 0,0,                     //Euler formular
                                                   0, Math.Cos(degree * theta), -Math.Sin(degree * theta), 0,
                                                   0, Math.Sin(degree * theta), Math.Cos(degree * theta), 0,
                                                   0,0,0,0);
            
            
            ObjectCoordinate = new SkeletonPoint[307200];
            for (int i = 0; i < 307200; i++)
            {
                real_world_point.X = ColorInSkel[i].X;
                real_world_point.Y = ColorInSkel[i].Y;
                real_world_point.Z = ColorInSkel[i].Z;
                NewCoordinate[i].X = (float)(Euler.M11 * real_world_point.X + Euler.M12 * real_world_point.Y + Euler.M13 * real_world_point.Z + Euler.M14);
                NewCoordinate[i].Y = (float)(Euler.M21 * real_world_point.X + Euler.M22 * real_world_point.Y + Euler.M23 * real_world_point.Z + Euler.M24);
                NewCoordinate[i].Z = (float)(Euler.M31 * real_world_point.X + Euler.M32 * real_world_point.Y + Euler.M33 * real_world_point.Z + Euler.M34);

            }
            x = NewCoordinate[(int)RMX + (int)RMY * 640].X;
            y = NewCoordinate[(int)RMX + (int)RMY * 640].Y;
            z = NewCoordinate[(int)RMX + (int)RMY * 640].Z;
            origin.X = x;
            origin.Y = y;
            origin.Z = z;
            transfer_origin(x,y,z);
            return real_world_point;
        }

        private Vector3D euler(int i)
        {
            double degree = Math.PI / 180, x, y, z;

            Vector3D real_world_point = new Vector3D();
            Matrix3D Euler = new Matrix3D(1, 0, 0, 0,                     //Euler formular
                                                   0, Math.Cos(degree * angel), -Math.Sin(degree * angel), 0,
                                                   0, Math.Sin(degree * angel), Math.Cos(degree * angel), 0,
                                                   0, 0, 0, 0);

            real_world_point.X = ColorInSkel[i].X;
            real_world_point.Y = ColorInSkel[i].Y;
            real_world_point.Z = ColorInSkel[i].Z;
            x = (float)(Euler.M11 * real_world_point.X + Euler.M12 * real_world_point.Y + Euler.M13 * real_world_point.Z + Euler.M14);
            y = (float)(Euler.M21 * real_world_point.X + Euler.M22 * real_world_point.Y + Euler.M23 * real_world_point.Z + Euler.M24);
            z = (float)(Euler.M31 * real_world_point.X + Euler.M32 * real_world_point.Y + Euler.M33 * real_world_point.Z + Euler.M34);
            real_world_point.X = x-origin.X;
            real_world_point.Y = y-origin.Y;
            real_world_point.Z = z-origin.Z;
            return real_world_point;
        }
        private Vector3D coordinate_transfer(double height_scale,double l)
        {
            Vector3D rwp = new Vector3D();
            string t;
            double theta,degree=180/Math.PI;
            theta = Math.Atan(l/height_scale)*degree ;
            angel = theta;
            if (framecount == 30)
            {
                t = "Theta: " + theta + " H: " + height_scale + " L: " + l;
                Console.WriteLine(t);
            }
            rwp = coordinate_rotate(theta);
           
            return rwp;
        
        }
        /*shift the kinect origin to floor(new origin)*/
        private Vector3D transfer_origin(double X, double Y, double Z)
        {
            Vector3D rwp = new Vector3D();
            Matrix3D T = new Matrix3D(1, 0, 0, -X,
                                       0, 1, 0, -Y,
                                       0, 0, 1, -Z,
                                       0, 0, 0, 1);
            for (int i = 0; i < 307200; i++)
            {
                NewCoordinate[i].X -= (float)X;
                NewCoordinate[i].Y -=(float)Y ;
                NewCoordinate[i].Z -= (float)Z;
            }
                
            
            return rwp;
        }
        #endregion
        private void SensorSkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            using (SkeletonFrame colorFrame = e.OpenSkeletonFrame())
            {

                if (colorFrame != null)
                {
                    if (transfer_value == true)
                    {

                        for (int i = 0; i < 640 * 480 * 4; i += 4)
                        {
                            if (NewCoordinate[i / 4].Y <= 0.05)
                            {
                                floorPixels[i] = 255;
                                floorPixels[i + 1] = 255;
                                floorPixels[i + 2] = 255;

                                d.Environment_label[i / 4] = 1;
                            }
                        }
                    }

                }
            }
        }

        private void SensorDepthFrameReady(object sender, DepthImageFrameReadyEventArgs e)
        {
            //   System.Drawing.Brush aa= System.Drawing.Brush(System.Drawing.Brushes.Red);
            double x = 0, y = 0, count = 0;
            short depth=0;
  #region environment build
            using (DepthImageFrame depthFrame = e.OpenDepthImageFrame())
            {
                test = 0;
                if (depthFrame != null)
                {
                    #region Initial
                    if (framecount == 0)
                    {
                        how_many_object = 0;
                        //  colorCoordinate = C();
                        for (int i = 0; i < this.depthPixels.Length; i++)
                        {
                            d.variation[i] = 0;
                            d.Environment_label[i] = 0;
                            d.mean[i] = 0;
                            d.counter[i] = 0;
                            d.ELB_ORG[i] = 0;

                        }
                        for (int i = 0; i < 100; i++)
                        {
                            object_center[i].X = 20;
                            object_center[i].Y = 20;
                            before_center[i].X = 20;
                            before_center[i].Y = 20;
                            center_cur[i].X = 0;
                            center_cur[i].Y = 0;
                            center_cur[i].Z = 0;

                            center_bef[i].X = 0;
                            center_bef[i].Y = 0;
                            center_bef[i].Z = 0;
                        }
                        d.depvalue.Initialize();
                    }
                    #endregion
                    // Copy the pixel data from the image to a temporary array
                    depthFrame.CopyDepthImagePixelDataTo(this.depthPixels);



                    //only the first 30 frames need to compute

                    #region background
                    if (framecount < 15)
                    {
                        for (int i = 0; i < this.depthPixels.Length; i++)
                        {
                            if (framecount < 10)
                            {
                                d.depvalue[framecount % 30, i] = depthPixels[i].Depth;
                                if (d.depvalue[framecount % 30, i] != 0)
                                {
                                    d.mean[i] += d.depvalue[framecount % 30, i];
                                    d.counter[i]++;
                                }
                            }
                            else if (framecount == 10)
                            {

                                if (d.counter[i] != 0)
                                    d.mean[i] /= d.counter[i];
                                else
                                    d.mean[i] = 0;
                                for (int j = 0; j < 10; j++)
                                {
                                    if (d.depvalue[j, i] != 0)
                                        d.variation[i] += Math.Pow((d.depvalue[j, i] - d.mean[i]), 2);
                                }
                                if (d.counter[i] != 0)
                                    d.variation[i] = (double)(Math.Sqrt(d.variation[i] / d.counter[i]));
                                else
                                    d.variation[i] = 30;
                            }
                        }
                    }

                    #endregion
                    if (transfer_value == true)
                    {
                        int colorPixelIndex = 0;
                        // co = 0;
                        #region lag reason


                        test = 0;
                        for (int i = 0; i < this.depthPixels.Length; i++)
                        {

                            depth = depthPixels[i].Depth;
                            this.backPixels[colorPixelIndex] = (byte)255;
                            this.backPixels[colorPixelIndex + 1] = (byte)255;
                            this.backPixels[colorPixelIndex + 2] = (byte)255;
                            /*
                             * d.Environment 1 is floor                 blue
                             *               2 is can't catch           red
                             *               3 is background            black
                             *               4 is foreground            white
                             *               5 is 1+3 print background  black
                             *               6 is 1+4 print foreground  white
                             *               7 is 1+2 print can't catch red
                             */
                            byte intensity = 0;
                            if (depth >= 80 && depth <= 6000)
                                intensity = (byte)depth;


                            if (framecount % 20 == 0)
                                multi_object_label[i] = 0;
                            if (d.variation[i] < 60)
                                d.variation[i] = 60;
                            if (intensity == 0)
                            {
                                if (framecount < 120)
                                    d.ELB_ORG[i] = 2;

                                if (d.ELB_ORG[i] != 2)
                                {
                                    if (framecount % 5 == 0)
                                        d.Environment_label[i] = 4;
                                    count++;
                                    if (framecount % 20 == 0)
                                    {
                                        if (find_eight(i, 4) >= 4)
                                        {
                                            multi_object_label[i] = -1;
                                        }
                                    }
                                    this.backPixels[colorPixelIndex] = (byte)255;
                                    this.backPixels[colorPixelIndex + 1] = (byte)255;
                                    this.backPixels[colorPixelIndex + 2] = (byte)255;

                                }
                                else
                                {
                                    if (framecount % 5 == 0)
                                        d.Environment_label[i] = 2;
                                    this.backPixels[colorPixelIndex] = 0;
                                    this.backPixels[colorPixelIndex + 1] = 0;
                                    this.backPixels[colorPixelIndex + 2] = (byte)255;
                                    test++;
                                }


                            }
                            else if (d.mean[i] == 0)
                            {
                                if (intensity != 0)
                                {
                                    if (framecount % 5 == 0)
                                        d.Environment_label[i] = 4;

                                    count++;
                                    if (framecount % 20 == 0)
                                    {
                                        if (find_eight(i, 4) >= 4)
                                        {
                                            multi_object_label[i] = -1;
                                        }
                                    }
                                    this.backPixels[colorPixelIndex] = (byte)255;
                                    this.backPixels[colorPixelIndex + 1] = (byte)255;
                                    this.backPixels[colorPixelIndex + 2] = (byte)255;

                                }
                                else
                                {
                                    if (framecount < 60)
                                        d.ELB_ORG[i] = 2;
                                    if (framecount % 5 == 0)
                                        d.Environment_label[i] = 2;
                                    this.backPixels[colorPixelIndex] = 0;
                                    this.backPixels[colorPixelIndex + 1] = 0;
                                    this.backPixels[colorPixelIndex + 2] = (byte)255;
                                    test++;
                                }
                            }

                            else if ((Math.Abs((double)depth - d.mean[i]) > (Math.Abs(d.variation[i] * 3) )))
                            {
                                // Vector3D a = new Vector3D();
                                //if(framecount%20==0)
                                // a = euler(i);
                                // if (false)
                                // {
                                //     this.backPixels[colorPixelIndex] = (byte)255;
                                //     this.backPixels[colorPixelIndex + 1] = 0;
                                //     this.backPixels[colorPixelIndex + 2] = 0;
                                //     d.Environment_label[i] = 1;
                                //     d.mean[i] += 0.2 * (depth - d.mean[i]);
                                // }
                                // else
                                // {
                                this.backPixels[colorPixelIndex] = (byte)255;
                                this.backPixels[colorPixelIndex + 1] = (byte)255;
                                this.backPixels[colorPixelIndex + 2] = (byte)255;
                                if (framecount % 5 == 0)
                                {
                                    d.Environment_label[i] = 4;
                                }
                                count++;
                                if (framecount % 20 == 0)
                                {
                                    if (find_eight(i, 4) >= 4)
                                    {
                                        multi_object_label[i] = -1;
                                    }
                                }
                                // }

                               
                            }
                            else if (d.Environment_label[i] == 1)
                            {
                                if (framecount % 5 == 0)
                                    d.Environment_label[i] = 1;
                                this.backPixels[colorPixelIndex] = (byte)255;
                                this.backPixels[colorPixelIndex + 1] = 0;
                                this.backPixels[colorPixelIndex + 2] = 0;
                            }

                            else if (Math.Abs((double)depth - d.mean[i]) <= (Math.Abs(d.variation[i] * 3) ) || d.Environment_label[i] == 3)
                            {


                                this.backPixels[colorPixelIndex] = 0;
                                this.backPixels[colorPixelIndex + 1] = 0;
                                this.backPixels[colorPixelIndex + 2] = 0;
                                if (framecount % 5 == 0)
                                    d.Environment_label[i] = 3;

                            }

                            //this.DepthPixels[colorPixelIndex] = intensity;
                            //// Write out green byte
                            //this.DepthPixels[colorPixelIndex + 1] = intensity;   
                            //// Write out red byte
                            //this.DepthPixels[colorPixelIndex + 2] = intensity;                          
                            colorPixelIndex += 4;
                        }
                      //  Console.WriteLine(test);
//                        printfdep(test.ToString());
                        

                        #endregion
                        if (framecount % 20 == 0 && transfer_value == true)
                        {
                            how_many_object = 0;
                            eight();
                        }
                        /*this.DepthBitmap.WritePixels(
                            new Int32Rect(0, 0, this.DepthBitmap.PixelWidth, this.DepthBitmap.PixelHeight),
                            this.DepthPixels,
                            this.DepthBitmap.PixelWidth * sizeof(int),
                            0);
                        */
                       


                            this.backdepthBitmap.WritePixels(
                                     new Int32Rect(0, 0, this.backdepthBitmap.PixelWidth, this.backdepthBitmap.PixelHeight),
                                     this.backPixels,
                                     this.backdepthBitmap.PixelWidth * sizeof(int),
                                     0);

                        Background = new ImageBrush(backdepthBitmap);
                    }
                    if (startlabel == true)
                    {
                        this.framecount++;
                    }
                    if (framecount == 300000)
                        framecount = 250;
                }
            }
#endregion
            #region draw the center

            using (DrawingContext dc = this.drawingGroup.Open())
            {

                if (transfer_value == true)
                {

                    if (framecount == 10)
                    {
                        Vector3D a = new Vector3D(0, 0, 0);
                        for (int i = 0; i < 200; i++)
                        {
                            f.fall_down_timer[i] = 0;
                            f.befor_speed[i] = 0;
                            f.acceleration[i] = 0;
                            f.now_speed[i] = 0;
                            f.now_center[i] = a;
                            f.before_center[i] = a;
                            f.spee_zero_timer[i] = 0;
                            f.speed_trig[i] = false;
                            f.object_timer[i] = 0;
                        }
                        for (int i = 0; i < 20; i++)
                        {
                            focus[i].hwratio_height = 0;
                            focus[i].hwratio_width= 0;
                            focus[i].toatal_area = 0;
                            focus[i].floor_area = 0;
                            focus[i].leftwidth = 0;
                            focus[i].mostwidth = 0;
                            focus[i].rightwidth = 0;
                            focus[i].vertical = 0;

                        }
                    }
                    dc.DrawRectangle(Background, null, new Rect(0.0, 0.0, 640.0, 480.0));
                    Vector3D center_cor = new Vector3D(0, 0, 0);
                    Vector3D center_cor2 = new Vector3D(0, 0, 0);
                    Vector3D current_cor = new Vector3D(0, 0, 0);
                    Vector3D before_cor = new Vector3D(0, 0, 0);


                    System.Windows.Point a1 = new System.Windows.Point(320, 240);

                    System.Windows.Point a2 = new System.Windows.Point((int)RMX, (int)RMY);
                    const int time_spent = 15;
                    int top = 1000, buttom = -1000, left = 1000, right = -1000;
                    dc.DrawEllipse(ColorBrush[2], null, a2, 10, 10);
                    int a22 = 0;



                    if (framecount % 15 == 0)
                    {
                        if (framecount % time_spent == 0)
                        {
                            t1 = null;
                            feature = null;
                            feature2 = null;
                            if (updating == false)
                                object_point.Clear();

                        }
                        //  int ttobj = 0;
                        curobject = 0;

                        if (framecount % time_spent == 0)
                        {
                            for (int i = 0; i < 100; i++)
                            {
                                center_cur[curobject].X = 0;
                                center_cur[curobject].Y = 0;
                                center_cur[curobject].Z = 0;

                            }
                        }
                        for (int i = 1; i < label_count; i++)
                        {
                            List<int> obj = new List<int>();
                            if (label_num[i] > 3000)
                            {

                                f.top[curobject] = 1000;
                                f.buttom[curobject] = -1000;
                                f.left[curobject] = 1000;
                                f.right[curobject] = -1000;
                                int x1 = 0, y1 = 0, count1 = 0, XX = 0, YY = 0, count2 = 0, lastJ = 0;
                                Boolean first = false;
                                double sY = 0, distance_cor = 0, distance_object = 0;
                                if (framecount % 30 == 0)
                                    count2 = 0;
                                int leftX = 10000, rightX = -10000;

                                focus[curobject].mostwidth = 0;
                                for (int j = 0; j < 640 * 480; j++)
                                {
                                    if (multi_object_label[j] == i)
                                    {
                                        obj.Add(j);
                                        x1 += j % 640;
                                        XX = j % 640;
                                        YY = j / 640;
                                        y1 += j / 640;
                                        if (first == false)
                                        {
                                            lastJ = j / 640;
                                            first = true;
                                        }
                                        if (YY < f.top[curobject])
                                            f.top[curobject] = YY;
                                        if (YY > f.buttom[curobject])
                                            f.buttom[curobject] = YY;
                                        if (XX > f.right[curobject])
                                            f.right[curobject] = XX;
                                        if (XX < f.left[curobject])
                                            f.left[curobject] = XX;

                                        if (framecount % time_spent == 0)
                                        {
                                            center_cor = euler(j);
                                            /*current*/
                                            if (center_cor.Y <= 2.0)
                                            {

                                                center_cur[curobject].X += center_cor.X;
                                                center_cur[curobject].Y += center_cor.Y;
                                                center_cur[curobject].Z += center_cor.Z;
                                                if (lastJ == j / 640)
                                                {
                                                    if (leftX > j % 640)
                                                        leftX = j % 640;
                                                    if (rightX < j % 640)
                                                        rightX = j % 640;
                                                }
                                                else if (lastJ != j / 640)
                                                {
                                                    focus[curobject].leftwidth = leftX;
                                                    focus[curobject].rightwidth = rightX;
                                                    if ((rightX - leftX) > focus[curobject].mostwidth)
                                                    {
                                                        focus[curobject].mostwidth = (rightX - leftX);
                                                        focus[curobject].leftw = lastJ * 640 + leftX;
                                                        focus[curobject].rightw = lastJ * 640 + rightX;
                                                    }
                                                    leftX = j % 640;
                                                    rightX = j % 640;
                                                }
                                                if (center_cor.Y <= 0.2) lower_point.Add(j);
                                                count2++;
                                            }
                                        }
                                        count1++;
                                        lastJ = j / 640;
                                    }
                                }
                                double most_H = -5000, most_L = 5000, leftwww = 2000, rightwww = -2000;
                                foreach (int z in obj)
                                {
                                    Vector3D a = euler(z);
                                    if (a.Y < 0.05 || a.Y > 2)
                                        continue;
                                    if (most_H < a.Y)
                                    {
                                        most_H = a.Y;
                                        focus[curobject].top_l = z;
                                    }
                                    if (most_L > a.Y)
                                    {
                                        most_L = a.Y;
                                        focus[curobject].low_l = z;
                                    }
                                    if (leftwww > a.X)
                                    {
                                        leftwww = a.X;
                                        focus[curobject].left_l = z;
                                    }
                                    if (rightwww < a.X)
                                    {
                                        rightwww = a.X;
                                        focus[curobject].right_l = z;
                                    }
                                }
                                focus[curobject].hwratio_height = most_H;
                                focus[curobject].mwbefore = focus[curobject].hwratio_width;
                                focus[curobject].hwratio_width = Math.Abs(rightwww - leftwww);
                                //   focus[curobject].leftw = leftwww;
                                //  focus[curobject].rightw = rightwww;
                                //if (most_L <= 0)
                                //most_L = 0;
                                focus[curobject].low = most_L;
                                if (count1 > 3000)
                                {
                                    if (focus[curobject].toatal_area == 0)
                                        focus[curobject].areabefore = 0;
                                    else
                                        focus[curobject].areabefore = Math.Round(((double)focus[curobject].floor_area / (double)focus[curobject].toatal_area), 3);
                                    focus[curobject].fabefore = focus[curobject].floor_area;
                                    focus[curobject].floor_area = lower_point.Count;
                                    focus[curobject].toatal_area = count1;
                                    /*current*/
                                    if (framecount > 30 && framecount % time_spent == 0)
                                    {
                                        before_center[curobject] = object_center[curobject];

                                        center_cor2 = euler((int)(before_center[curobject].X + before_center[curobject].Y * 640));
                                    }
                                    object_center[curobject].X = (int)(x1 / count1);
                                    object_center[curobject].Y = (int)(y1 / count1);
                                    center_cor = euler((int)(object_center[curobject].X + object_center[curobject].Y * 640));
                                    if (framecount % time_spent == 0)
                                    {
                                        center_cur[curobject].X /= count2;
                                        center_cur[curobject].Y /= count2;
                                        center_cur[curobject].Z /= count2;
                                        focus[curobject].vertical = center_cur[curobject].Y;
                                    }
                                    /*compute the dsitance and now become the before*/


                                    if (framecount > 30 && framecount % time_spent == 0)
                                    {
                                        distance_cor = Math.Pow((center_cur[curobject].X - center_bef[curobject].X), 2) + Math.Pow((center_cur[curobject].Y - center_bef[curobject].Y), 2) + Math.Pow((center_cur[curobject].Z - center_bef[curobject].Z), 2);
                                        distance_cor = Math.Sqrt(distance_cor);


                                        // distance_object = Math.Pow((before_center[curobject].X - object_center[curobject].X), 2) + Math.Pow((before_center[curobject].Y - object_center[curobject].Y), 2);
                                        //distance_object = Math.Sqrt(distance_object);
                                        distance_object = Math.Pow((center_cor.X - center_cor2.X), 2) + Math.Pow((center_cor2.Y - center_cor.Y), 2) + Math.Pow((center_cor2.Z - center_cor.Z), 2);
                                        distance_object = Math.Sqrt(distance_object);
                                        // center_cor=euler((int)obejec_center[curobject].X+(int)obejec_center[curobject].Y*640);

                                    }

                                    if (framecount % time_spent == 0)
                                    {
                                        sY = (double)center_cur[curobject].Y - center_bef[curobject].Y;
                                        focus[curobject].vbefore = center_bef[curobject].Y;

                                        t1 += "No.: " + (curobject + 1) + "\n";
                                        t1 += "Timer: " + f.fall_down_timer[curobject] + "\n";
                                        t1 += "Speed_timer: " + f.spee_zero_timer[curobject] + "\n";
                                        t1 += "Object_timer: " + f.object_timer[curobject] + "\n";
                                        t1 += "Distance: " + distance_cor * 100 + "cm\n\n";
                                        t1 += "-------------Speed-----------------\n";
                                        t1 += "Acceleration: " + (sY - speed[curobject]) * 100 / (time_spent / 30) / (time_spent / 30) + "  cm/s^2\n";
                                        t1 += "Speed_bef: " + speed[curobject] * 100 / (time_spent / 30) + "  cm/s\n";
                                        t1 += "Speed_cur: " + sY * 100 / (time_spent / 30) + " cm/s\n";

                                        feature += "No.: " + (curobject + 1) + "\n";
                                        feature += "Hgih: " + (Math.Abs(f.top[curobject] - f.buttom[curobject])) + "\n";
                                        feature += "Weight: " + (Math.Abs(f.right[curobject] - f.left[curobject])) + "\n";

                                        feature += "Acceleration: " + (sY - speed[curobject]) * 100 / (time_spent / 30) / (time_spent / 30) + "  cm/s^2\n";
                                        feature += "Speed_bef: " + speed[curobject] * 100 / (time_spent / 30) + "  cm/s\n";
                                        feature += "Speed_cur: " + sY * 100 / (time_spent / 30) + " cm/s\n";

                                        feature2 += "No.: " + (curobject + 1) + "\n";
                                        feature2 += "Area: " + focus[curobject].toatal_area + "\n";
                                        feature2 += "Vertical: " + Math.Round(focus[curobject].vertical, 3) * 100 + "cm\n";
                                        feature2 += "Wmax Pixel: " + focus[curobject].mostwidth + "\n";
                                        //feature2 += "W Pixel: " + (f.right[curobject] - f.left[curobject]) + "\n";
                                        feature2 += "Hmax Pixel: " + (-f.top[curobject] + f.buttom[curobject]) + "\n";
                                        feature2 += "Width : " + Math.Round(focus[curobject].hwratio_width, 3) * 100 + "cm\n";

                                        //feature2 += "WL : " + Math.Round(euler(f.left[curobject]+10).Y, 3) * 100 + "cm\n";
                                        // feature2 += "WR : " + Math.Round(euler(f.right[curobject]-10).Y, 3) * 100 + "cm\n";

                                        //feature2 += "Wright: " + focus[curobject].rightwidth + "\n";
                                        feature2 += "Height: " + Math.Round(focus[curobject].hwratio_height, 3) * 100 + "cm\n";
                                        feature2 += "Low: " + Math.Round(focus[curobject].low, 3) * 100 + "cm\n";
                                        feature2 += "H/W ratio: " + Math.Round(focus[curobject].hwratio_height / focus[curobject].hwratio_width, 3) + "\n";
                                        feature2 += "H/W pixel: " + Math.Round(((double)(-f.top[curobject] + f.buttom[curobject])) / focus[curobject].mostwidth, 3) + "\n";
                                        feature2 += "Total Point: " + count1 + "\n";
                                        feature2 += "Lower Point: " + lower_point.Count + "\n";
                                        feature2 += "Lower %: " + Math.Round(((double)((double)lower_point.Count / (double)count1)), 3) * 100 + "%\n";

                                        //   feature2 += "left:" + Math.Round(focus[curobject].leftw, 3);
                                        //  feature2 += "right:" + Math.Round(focus[curobject].rightw, 3);
                                        //if ((sY > speed[curobject]) && (sY - speed[curobject]) > 10 / 100)
                                        //    feature += "Move_direction: up \n";
                                        //else if ((sY < speed[curobject]) && (speed[curobject] - sY) > 10 / 100)
                                        //    feature += "Move_direction: down \n";
                                        //else
                                        //    feature += "Move_direction: stable \n";
                                        //  Console.WriteLine("(" + f.left[curobject] + "," + f.top[curobject] + ")" + "(" + f.right[curobject] + "," + f.buttom[curobject] + ")");
                                        //     Console.WriteLine(object_center[curobject].X+","+object_center[curobject].Y+","+distance_object*100);

                                        if (f.acceleration[curobject] < -1 && (sY / (time_spent / 30) - speed[curobject]) * 100 / (time_spent / 30) / (time_spent / 30) > 1 && ((sY - speed[curobject]) * 100 / (time_spent / 30) / (time_spent / 30) - f.acceleration[curobject]) >= 5)
                                            f.speed_trig[curobject] = true;

                                        f.befor_speed[curobject] = speed[curobject] * 100 / (time_spent / 30);
                                        f.now_speed[curobject] = sY * 100 / (time_spent / 30);
                                        f.acceleration[curobject] = (sY / (time_spent / 30) - speed[curobject]) * 100 / (time_spent / 30) / (time_spent / 30);
                                        f.before_center[curobject] = center_bef[curobject];
                                        // f.now_center[curobject] = center_cur[curobject];
                                        center_cor = euler((int)(object_center[curobject].X + object_center[curobject].Y * 640));
                                        f.now_center[curobject] = center_cor;

                                        //if (f.spee_zero_timer[curobject] >= 12&&f.now_center[curobject].Y<=0.5)
                                        //    Console.WriteLine(", FALL");
                                        //else if (f.spee_zero_timer[curobject] > 0)
                                        //    Console.WriteLine(", Warn");
                                        //else
                                        //    Console.WriteLine();
                                        t1 += "Time spent: 1 sec\n";
                                        t1 += "-------------------------------------\n\n";
                                        t1 += "X difference: " + (center_cur[curobject].X * 100 - center_bef[curobject].X * 100) + "cm\n";
                                        t1 += "Y difference: " + (center_cur[curobject].Y * 100 - center_bef[curobject].Y * 100) + "cm\n";
                                        t1 += "Z difference: " + (center_cur[curobject].Z * 100 - center_bef[curobject].Z * 100) + "cm\n";

                                        t1 += "\n--------one second before------\n";
                                        t1 += "center_before.X: " + center_bef[curobject].X * 100 + "cm\n";
                                        t1 += "center_before.Y: " + center_bef[curobject].Y * 100 + "cm\n";
                                        t1 += "center_before.Z: " + center_bef[curobject].Z * 100 + "cm\n";
                                        t1 += "-------------------------------------\n";

                                        feature += "Center X: " + center_cur[curobject].X + "\n";
                                        feature += "Center Y: " + center_cur[curobject].Y + "\n";
                                        feature += "Center Z: " + center_cur[curobject].Z + "\n";

                                        speed[curobject] = sY;

                                        /*before*/
                                        center_bef[curobject] = center_cur[curobject];

                                        if (distance_cor <= 0.15)
                                        {
                                            if ((center_bef[curobject].Y) <= 0.5)
                                                f.fall_down_timer[curobject]++;
                                            else
                                                f.fall_down_timer[curobject] = 0;
                                        }
                                        else
                                            f.fall_down_timer[curobject] = 0;

                                        if (distance_object <= 0.05)
                                            f.object_timer[curobject]++;
                                        else
                                            f.object_timer[curobject] = 0;

                                        //if (f.befor_speed[curobject] < -10 && (f.now_speed[curobject] >= -3&&f.now_speed[curobject] <= 3)&&(f.now_speed[curobject]-f.befor_speed[curobject])>=0)
                                        //{
                                        //    f.speed_trig[curobject] = true;
                                        //}

                                        if (f.speed_trig[curobject] == true && f.now_speed[curobject] >= -3 && f.now_speed[curobject] <= 3)
                                            f.spee_zero_timer[curobject]++;
                                        else
                                        {
                                            f.speed_trig[curobject] = false;
                                            f.spee_zero_timer[curobject] = 0;
                                        }

                                        t1 += "-------one second after----------\n";
                                        t1 += "center_after.X: " + center_cur[curobject].X * 100 + " cm\n";
                                        t1 += "center_after.Y: " + center_cur[curobject].Y * 100 + " cm\n";
                                        t1 += "center_after.Z: " + center_cur[curobject].Z * 100 + " cm\n";
                                        t1 += "--------------------------------------\n\n";

                                        // if(framecount%30==0)
                                        if (updating == false)
                                            object_point.Add(obj);
                                        //else
                                        //    f.object_timer[curobject] = 0;
                                        //lower_point.Clear();

                                        feature += "Total Point: " + count1 + "\n";
                                        feature += "Lower Point: " + lower_point.Count + "\n";
                                        feature += "Lower %: " + Math.Round(((double)((double)lower_point.Count / (double)count1)), 3) * 100 + "%\n";
                                        lower_point.Clear();
                                    }
                                    curobject++;
                                    // ttobj++;
                                }
                            }
                            else if (label_num[i] > 100)
                            {
                                if (framecount % 30 == 0)
                                {
                                    List<int> obj2 = new List<int>();
                                    for (int j = 0; j < 640 * 480; j++)
                                    {
                                        if (multi_object_label[j] == i)
                                        {
                                            obj2.Add(j);
                                        }
                                    }
                                    if (object_point.Count < 20)
                                        object_point.Add(obj2);
                                }

                            }
                            printfdep(feature2);
                        }

                        

                        double yes = center_cur[0].Y * 100 - focus[0].vbefore * 100;

                        variable_V += yes;

                        yes = (Math.Round(((double)focus[0].floor_area / (double)focus[0].toatal_area), 3) - focus[0].areabefore) * 100;
                        if (focus[0].toatal_area == 0 || focus[0].areabefore == 0)
                            yes = 0;
                        variable_R += yes;
                        yes = (focus[0].hwratio_width * 100 - focus[0].mwbefore * 100);
                        variable_W += yes;

                        yes = focus[0].floor_area - focus[0].fabefore;
                        variable_A += yes;

                       // ploti++;
                    }
                    //if (framecount % slide_framecount == 0)
                    //{
                    //    vari_ploti++;
                    //}
                    for (int i = 0; i < object_point.Count - 1; i++)
                    {
                        List<int> temp = new List<int>();
                        for (int j = i + 1; j < object_point.Count; j++)
                        {
                            if (object_point[i].Count < object_point[j].Count)
                            {
                                temp = object_point[i];
                                object_point[i] = object_point[j];
                                object_point[j] = temp;
                            }
                        }
                    }
                    #region update
                    for (int i = 0; i < curobject; i++)
                    {
                        if (f.fall_down_timer[i] >= 6 * 30 / time_spent&&f.now_center[i].Y>=0.1)
                        {
                            dc.DrawEllipse(ColorBrush[3], null, a2, 30, 30);
                            //  break;
                        }
                        if (f.spee_zero_timer[i] >= 6 * 30 / time_spent && f.now_center[i].Y <= 0.5&&f.now_center[i].Y>=0.1)
                        {
                            dc.DrawEllipse(ColorBrush[7], null, new System.Windows.Point(50,50), 30, 30);
                            //break;
                        }
                        if (f.object_timer[i] >= 15 * 30 / time_spent)
                        {
                            if (f.now_center[i].Y <= 0.05)
                            {
                                updating = true;
                                update_num = i;
                                
                            }
                            else if (updating != true && curobject == 1)
                            {
                                updating = false;
                            }
                            else
                            {
                                updating = false;
                                update_num = 20;
                            }
                        }
                        if (update_num != 20 && updating == true)
                            break;
                    }
                    
                    if(false)//if (updating == true)
                    {
                        coming_time++;
                        /*only for the object on the floor*/
                        for (int k = 0; k < object_point[update_num].Count; k++)
                        {
                            if (object_point[update_num][k] > 640 && object_point[update_num][k] < 639 * 479 && (object_point[update_num][k] - 2) % 640 != 0 && (object_point[update_num][k] + 2) % 640 != 0)
                            d.mean[object_point[update_num][k]] += (this.depthPixels[object_point[update_num][k]].Depth - d.mean[object_point[update_num][k]]) * 0.01;
                            if (coming_time % 45 == 0)
                            {

                                Vector3D a = new Vector3D();
                                a = euler(object_point[update_num][k]);
                                NewCoordinate[object_point[update_num][k]].X = (float)a.X;
                                NewCoordinate[object_point[update_num][k]].Y = (float)a.Y;
                                NewCoordinate[object_point[update_num][k]].Z = (float)a.Z;
                                if (a.Y > 0.05)
                                    d.Environment_label[object_point[update_num][k]] = 3;
                                else
                                    d.Environment_label[object_point[update_num][k]] = 1;
                            }

                        }
                        if (coming_time % 300 == 0)
                        {
                            update_num = 0;
                            updating = false;
                            coming_time = 0;
                        }

                    }

                    /* for (int i = curobject; i < object_point.Count; i++)
                     {
                         for (int j = 0; j < object_point[i].Count; j++)
                         {
                             if (object_point[i][j] > 640 && object_point[i][j] < 639 * 479 && (object_point[i][j] - 2) % 640 != 0 && (object_point[i][j] + 2) % 640 != 0)
                                  d.mean[object_point[i][j]] += (this.depthPixels[object_point[i][j]].Depth - d.mean[object_point[i][j]]) * 0.01;
                             if (framecount % 30 == 0)
                             {
                                 Vector3D a = new Vector3D();
                                 a = euler(object_point[i][j]);
                                 NewCoordinate[object_point[i][j]].X = (float)a.X;
                                 NewCoordinate[object_point[i][j]].Y = (float)a.Y;
                                 NewCoordinate[object_point[i][j]].Z = (float)a.Z;
                                 if (a.Y > 0.05)
                                     d.Environment_label[object_point[i][j]] = 3;
                                 else
                                     d.Environment_label[object_point[i][j]] = 1;
                             }
                         }
                     }*/
                    //new System.Windows.Point(f.left[0],f.top[0]),new System.Windows.Point(f.right[0],f.buttom[0]);



                    //dc.DrawLine(new System.Windows.Media.Pen(ColorBrush[1], 3), obejec_center[0], new System.Windows.Point(f.right[0], f.buttom[0]));
                    //dc.DrawLine(new System.Windows.Media.Pen(ColorBrush[1], 3), obejec_center[0], new System.Windows.Point(f.left[0], f.buttom[0]));
                    //dc.DrawLine(new System.Windows.Media.Pen(ColorBrush[1], 3), obejec_center[0], new System.Windows.Point(f.left[0], f.top[0]));
                    //dc.DrawLine(new System.Windows.Media.Pen(ColorBrush[1], 3), obejec_center[0], new System.Windows.Point(f.right[0], f.top[0]));
                    #endregion
                    #region draw many
                    if (curobject != 0)
                    {
                        dc.DrawEllipse(ColorBrush[0], null, object_center[0], 10, 10);
                        dc.DrawEllipse(ColorBrush[1], null, object_center[1], 10, 10);
                        dc.DrawEllipse(ColorBrush[7], null, object_center[2], 10, 10);
                        dc.DrawEllipse(ColorBrush[3], null, object_center[3], 10, 10);
                        dc.DrawEllipse(ColorBrush[4], null, object_center[4], 10, 10);
                        dc.DrawEllipse(ColorBrush[7], null, object_center[5], 10, 10);
                        dc.DrawLine(new System.Windows.Media.Pen(ColorBrush[2], 5), new System.Windows.Point(focus[0].rightw%640, focus[0].rightw/640), new System.Windows.Point(focus[0].leftw%640, focus[0].leftw/640));
                        dc.DrawEllipse(ColorBrush[7], null, new System.Windows.Point(focus[0].low_l % 640, focus[0].low_l / 640), 10, 10);
                        dc.DrawEllipse(ColorBrush[3], null, new System.Windows.Point(focus[0].right_l % 640, focus[0].right_l / 640), 10, 10);
                        dc.DrawEllipse(ColorBrush[7], null, new System.Windows.Point(focus[0].top_l % 640, focus[0].top_l / 640), 10, 10);
                        dc.DrawEllipse(ColorBrush[3], null, new System.Windows.Point(focus[0].left_l % 640, focus[0].left_l / 640), 10, 10);
                        dc.DrawRectangle(null, new System.Windows.Media.Pen(ColorBrush[1], 5), new Rect(new System.Windows.Point(f.left[0], f.top[0]), new System.Windows.Point(f.right[0], f.buttom[0])));
                        dc.DrawRectangle(null, new System.Windows.Media.Pen(ColorBrush[1], 5), new Rect(new System.Windows.Point(f.left[1], f.top[1]), new System.Windows.Point(f.right[1], f.buttom[1])));
                        dc.DrawRectangle(null, new System.Windows.Media.Pen(ColorBrush[1], 5), new Rect(new System.Windows.Point(f.left[2], f.top[2]), new System.Windows.Point(f.right[2], f.buttom[2])));
                        dc.DrawRectangle(null, new System.Windows.Media.Pen(ColorBrush[1], 5), new Rect(new System.Windows.Point(f.left[3], f.top[3]), new System.Windows.Point(f.right[3], f.buttom[3])));
                        dc.DrawRectangle(null, new System.Windows.Media.Pen(ColorBrush[1], 5), new Rect(new System.Windows.Point(f.left[4], f.top[4]), new System.Windows.Point(f.right[4], f.buttom[4])));
                        dc.DrawRectangle(null, new System.Windows.Media.Pen(ColorBrush[1], 5), new Rect(new System.Windows.Point(f.left[5], f.top[5]), new System.Windows.Point(f.right[5], f.buttom[5])));
                    }
                    #endregion
                    #region struct reset
                    for (int i = curobject; i < 100; i++)
                    {
                        Vector3D a = new Vector3D(0, 0, 0);

                        f.fall_down_timer[i] = 0;
                        f.befor_speed[i] = 0;
                        f.acceleration[i] = 0;
                        f.now_speed[i] = 0;
                        f.before_center[i] = a;
                        f.spee_zero_timer[i] = 0;
                        f.speed_trig[i] = false;
                        f.object_timer[i] = 0;

                        object_center[i].X = 20;
                        object_center[i].Y = 20;
                        f.buttom[i] = 0;
                        f.top[i] = 0;
                        f.right[i] = 0;
                        f.left[i] = 0;
                        f.now_center[i] = a;


                    }
                    #endregion
                }
                #endregion
            }
        }
        #region no need to change
        private int find_eight(int i,int lab)
        {
            int count = 0;
            if (i > 640 && i < 639 * 479 && (i - 2) % 640 != 0 && (i + 2) % 640 != 0)
            {
                if ((d.Environment_label[i + 1] == lab))
                    count++;
                if (d.Environment_label[i - 1] == lab)
                    count++;
                if (d.Environment_label[i + 640 + 1] == lab )
                    count++;
                if (d.Environment_label[i - 1 - 640] == lab )
                    count++;
                if (d.Environment_label[i + 640] == lab )
                    count++;
                if (d.Environment_label[i + 1] == lab )
                    count++;
                if (d.Environment_label[i - 640 + 1] == lab )
                    count++;
                if (d.Environment_label[i - 640] == lab)
                    count++;

                return count;
            }
            else
                return 0;
        }
        private void eight()
        {
            
            int min=1000;
            label_count = 0;
            
            
          //  System.IO.StreamWriter sw = new System.IO.StreamWriter("data.csv"); // open the file for streamwriter
           
            for(int i=0;i<640*480;i++)
            {
                label_group[i] = i;
                label_num[i] = 0;
                min = 1000;
                
                if (i > 640 && i < 639 * 479 && (i - 2) % 640 != 0 && (i + 2) % 640 != 0)
                {
                    if (multi_object_label[i] == -1)
                    {
                        if (multi_object_label[i - 1] > 0)
                        {
                            if (min > multi_object_label[i - 1])
                                min = multi_object_label[i - 1];
                        }
                        if (multi_object_label[i - 640 - 1] > 0)
                        {
                            if (min > multi_object_label[i - 1 - 640])
                                min = multi_object_label[i - 1 - 640];
                        }

                        if (multi_object_label[i - 640] > 0)
                        {
                            if (min > multi_object_label[i - 640])
                            {
                                min = multi_object_label[i - 640];
                            }

                        }

                        if (multi_object_label[i - 640 + 1] > 0)
                        {
                            if (min > multi_object_label[i - 640 + 1])
                                min = multi_object_label[i - 640 + 1];
                        }

                        if (min != 1000)
                        {
                            multi_object_label[i] = min;
                        }
                        else
                        {
                            multi_object_label[i] = ++label_count;
                        }
                         merge(multi_object_label[i - 1], multi_object_label[i - 641], multi_object_label[i - 640], multi_object_label[i - 639]);
                        //label_num[multi_object_label[i]]++;
                    }
                }

            }

            for (int i = 1; i < label_count; i++)
            {
                int root;
                root = find_root(label_group[i]);
                label_group[i] = root;
            }
            
            for (int i = 0; i < 640 * 480; i++)
            {
                if (i > 640 && i < 639 * 479 && (i - 2) % 640 != 0 && (i + 2) % 640 != 0)
                {
                    if (multi_object_label[i] > 0)
                    {
                        multi_object_label[i] = label_group[multi_object_label[i]];

                        label_num[multi_object_label[i]]++;
                    }
                }
            }

            //if(framecount%300==0)
            //    for (int i = 0; i < 640 * 480; i++)
            //    {
            //        if (i % 640 == 0)
            //            Console.WriteLine();
            //        Console.Write(multi_object_label[i]); // output the reslut to the file (WriteLine or Write)
            //        Console.Write(" ");
            //    }

           // sw.Close(); // close the file
            
           // Console.WriteLine("-----------------------------------");
            for (int i = 0; i < label_count; i++)
            {
              //  Console.WriteLine(label_num[i]);
                if (label_num[i] > 3000)
                    how_many_object++;
            }
          //  Console.WriteLine("-----------------------------------");
            //string t = null;
            //t += "Object_num: " + how_many_object + "\n";
            //printfdep(t);
        }
        private void merge(int a, int b, int c, int d)
        {
            int min = 10000;
            if (a != 0)
            {
                if (min > a)
                    min = a;
            }
            if (b != 0)
            {
                if (min > b)
                    min = b;
            }
            if (c != 0)
            {
                if (min > c)
                    min = c;
            }
            if (d != 0)
            {
                if (min > d)
                    min = d;
            }
            
            if (a != 0)
            {
                if(label_group[a] > min)
                label_group[a] = min;
            }
            if (b != 0)
            {
                if (label_group[b] > min)
                    label_group[b] = min;
            }
            if (c != 0)
            {
                if (label_group[c] > min)
                    label_group[c] = min;
            }
            if (d != 0)
            {
                if (label_group[d] > min)
                    label_group[d] = min;
            }
        }

        private void slider_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            slide_framecount = (int)slider.Value;
            timer2.Interval = TimeSpan.FromSeconds((double)(slide_framecount / 30));
            timer2.Interval = TimeSpan.FromSeconds((double)(slide_framecount / 30));
            timer2.Interval = TimeSpan.FromSeconds((double)(slide_framecount / 30));
            timer2.Interval = TimeSpan.FromSeconds((double)(slide_framecount / 30));
            try
            {
                label2.Content = slide_framecount.ToString();
            }
            catch (Exception e1)
            { }

        }
        private int find_root(int index)
        {
            int i = index;
            while (i != label_group[i])
            {
                i = label_group[i];
            }
            return i;
            //if (label_group[index] == index)
            //    return index;
            //else
            //    return find_root(label_group[index]);
        }
        private System.Windows.Point SkeletonPointToScreen(SkeletonPoint skelpoint)
        {
            DepthImagePoint depthPoint = this.sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skelpoint, DepthImageFormat.Resolution640x480Fps30);
            return new System.Windows.Point(depthPoint.X, depthPoint.Y);
        }
        public SkeletonPoint[] ColorToSkel(int W, int H)
        {
            SkeletonPoint[] SKK = new SkeletonPoint[W * H];
            this.sensor.CoordinateMapper.MapColorFrameToSkeletonFrame(ColorImageFormat.RgbResolution640x480Fps30, DepthImageFormat.Resolution640x480Fps30, this.depthPixels, SKK);
            return SKK;
        }
        public ColorImagePoint[] C()
        {
            ColorImagePoint[] a = new ColorImagePoint[640 * 480];
            this.sensor.CoordinateMapper.MapDepthFrameToColorFrame(DepthImageFormat.Resolution640x480Fps30, this.depthPixels, ColorImageFormat.RgbResolution640x480Fps30, a);
            return a;
        }
        private void Image_MouseLeftButtonDown(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            MouseX = (int)e.GetPosition(Image).X;
            MouseY = (int)e.GetPosition(Image).Y;
            Xr = ColorInSkel[MouseX + MouseY * 640].X;
            Yr = ColorInSkel[MouseX + MouseY * 640].Y;
            Zr = ColorInSkel[MouseX + MouseY * 640].Z;

            if (transfer_value == true)
            {
                
                Xr = NewCoordinate[MouseX + MouseY * 640].X;
                Yr = NewCoordinate[MouseX + MouseY * 640].Y;
                Zr = NewCoordinate[MouseX + MouseY * 640].Z;
                //for (int i = 0; i < 307200; i++)
                //{
                //    double errY = NewCoordinate[i].Y - NewCoordinate[MouseX + MouseY * 640].Y;
                //    double errX = NewCoordinate[i].X - NewCoordinate[MouseX + MouseY * 640].X;
                //    double errZ = NewCoordinate[i].Z - NewCoordinate[MouseX + MouseY * 640].Z;
                //    if (Math.Abs(errY) <= 0.05&&Math.Abs(errX)<=0.6&&Math.Abs(errZ)<=0.6)
                //    {
                //        object_point.Add(i);
                //    }
                
                //}
                //object_list_count = object_point.Count - object_list_count;
                //Console.WriteLine(object_point.Count);
                //object_count++;
                
                    string result = null;
                    MouseX = (int)e.GetPosition(Image).X;
                    MouseY = (int)e.GetPosition(Image).Y;
                    Vector3D a = euler(MouseX + MouseY * 640);
                    result += "X: " + a.X + "\nY: " + a.Y + "\nZ: " + a.Z;
                    printfdep(result);
                
            }

             Console.Write("Real World Point ==== X:");
             Console.Write(Xr);
             Console.Write(" Y: ");
             Console.Write(Yr);
             Console.Write(" Z: ");
             Console.WriteLine(Zr);
      
        }
        private void Image_MouseRightButtonDown(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            if (transfer_value == false)
            {
                double X, Y, Z;
                RMX = (int)e.GetPosition(Image).X;      //right click down X will become to the new origin
                RMY = (int)e.GetPosition(Image).Y;      //right click down Y
                X = ColorInSkel[(int)RMX + (int)RMY * 640].X;
                Y = ColorInSkel[(int)RMX + (int)RMY * 640].Y;
                Z = ColorInSkel[(int)RMX + (int)RMY * 640].Z;


                string text1;
                Console.WriteLine("============BEFORE=========");
                text1 = "X: " + X + " Y: " + Y + " Z: " + Z;
                Console.WriteLine(text1);
            }
            

        }
        private void Image_MouseRightButtonUp(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            if (transfer_value == false)
            {
                string t;
                double X, Y, Z;
                X = ColorInSkel[(int)e.GetPosition(Image).X + (int)e.GetPosition(Image).Y * 640].X;
                Y = ColorInSkel[(int)e.GetPosition(Image).X + (int)e.GetPosition(Image).Y * 640].Y;
                Z = ColorInSkel[(int)e.GetPosition(Image).X + (int)e.GetPosition(Image).Y * 640].Z;

                string text1;
                Console.WriteLine("============AFTER=========");
                text1 = "X: " + X + " Y: " + Y + " Z: " + Z;
                Console.WriteLine(text1);
                Console.WriteLine("============END=========");
                Right_lengthX = Math.Abs(ColorInSkel[(int)RMX + (int)RMY * 640].X - X);

                if (Right_lengthX >= 0.0)
                {
                    // Console.WriteLine(Z);
                    Z = ColorInSkel[(int)RMX + (int)e.GetPosition(Image).Y * 640].Z;
                    //  Console.WriteLine(Z);
                }
                Right_lengthY = Math.Abs(ColorInSkel[(int)RMX + (int)RMY * 640].Y - Y);
                Right_lengthZ = Math.Abs(ColorInSkel[(int)RMX + (int)RMY * 640].Z - Z);
                t = "X: " + Right_lengthX;
                Console.WriteLine(t);
                t = "Y: " + Right_lengthY;
                Console.WriteLine(t);
                t = "Z: " + Right_lengthZ;
                Console.WriteLine(t);

                coordinate_transfer(Right_lengthZ, Right_lengthY);
                transfer_value = true;
            }
        }
      /*  private void Image2_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            double test;
            string result = null;
            MouseX = (int)e.GetPosition(Image2).X;
            MouseY = (int)e.GetPosition(Image2).Y;
            test = this.depthPixels[MouseX + MouseY * 640].Depth;
            //Console.Write("Depth value:");
            //Console.WriteLine(test);
            //for (int i = 0; i < 30; i++)
            //{
            //    test = d.depvalue[i, (MouseX + MouseY * 640)];
            //    //test = this.meandep[(MouseX + MouseY * 640)];

            //    result += i + "th  Frames  Depth value : " + test + "\n";
            //}
            //test = d.counter[MouseX + MouseY * 640];
            //result += framecount % 30 + " Frames count Depth value: " + test + "\n";
            //test = d.mean[MouseX + MouseY * 640];
            //result += framecount % 30 + " Frames means Depth value: " + test + "\n";
            //test = d.variation[MouseX + MouseY * 640];
            //result += framecount % 30 + " Frames variation Depth value: " + test + "\n";
            result += "X: " + NewCoordinate[MouseX + MouseY * 640].X + "\nY: " + NewCoordinate[MouseX + MouseY * 640].Y + "\nZ: " + NewCoordinate[MouseX + MouseY * 640].Z;
            printfdep(result);
        }
        */
        private void Image4_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            double test;
            string result = null;
            MouseX = (int)e.GetPosition(Image4).X;
            MouseY = (int)e.GetPosition(Image4).Y;
            Vector3D a = euler(MouseX + MouseY * 640);
//            test = this.depthPixels[MouseX + MouseY * 640].Depth;
            //Console.Write("Depth value:");
            //Console.WriteLine(test);
            //for (int i = 0; i < 30; i++)
            //{
            //    test = d.depvalue[i, (MouseX + MouseY * 640)];
            //    //test = this.meandep[(MouseX + MouseY * 640)];

            //    result += i + "th  Frames  Depth value : " + test + "\n";
            //}
            //test = d.counter[MouseX + MouseY * 640];
            //result += framecount % 30 + " Frames count Depth value: " + test + "\n";
            //test = d.mean[MouseX + MouseY * 640];
            //result += framecount % 30 + " Frames means Depth value: " + test + "\n";
            //test = d.variation[MouseX + MouseY * 640];
            //result += framecount % 30 + " Frames variation Depth value: " + test + "\n";
            result += "X: " + a.X + "\nY: " + a.Y + "\nZ: " + a.Z;
            printfdep(result);
        }
        private void printfdep(string t)
        {
            textBox1.FontSize = 14;
            textBox1.Text = t;
        }
        private void button1_Click(object sender, RoutedEventArgs e)
        {
            startlabel = true;
        }
        private void button2_Click(object sender, RoutedEventArgs e)
        {
            
            for (int i = 0; i < 640 * 480; i++)
            {
                NewCoordinate[i].X = 0;
                NewCoordinate[i].Y = 0;
                NewCoordinate[i].Z = 0;
                
            }
            transfer_value = false;
            startlabel = false;
            framecount = 0;
        }
        #endregion
        private void AnimatedPlot_center(object sender, EventArgs e)
        {
            if (transfer_value == true)
            {
                double y = center_cur[0].Y * 100;
                if (y == 0)
                    y = 0;
                System.Windows.Point Pl_p = new System.Windows.Point(ploti, y);
                dataSource1.AppendAsync(base.Dispatcher, Pl_p);
               // ploti++;
            }
        }
        private void AnimatedPlot_hwratio(object sender, EventArgs e)
        {
            if (transfer_value == true)
            {
                double y = Math.Round((double)focus[0].floor_area/(double)focus[0].toatal_area,3)*100;
                if (focus[0].toatal_area == 0)
                    y = 0;
                System.Windows.Point Pl_p = new System.Windows.Point(ploti, y);
                dataSource2.AppendAsync(base.Dispatcher, Pl_p);
               // ploti++;
            }
        }
        private void AnimatedPlot_widthest(object sender, EventArgs e)
        {
            if (transfer_value == true)
            {
                double y = (focus[0].hwratio_width * 100);
                if (y == 0)
                    y = 0;
                System.Windows.Point Pl_p = new System.Windows.Point(ploti, y);
                dataSource3.AppendAsync(base.Dispatcher, Pl_p);
               // ploti++;
            }
        }
        private void AnimatedPlot_area(object sender, EventArgs e)
        {
            if (transfer_value == true)
            {
                double y = focus[0].floor_area;
                if (y == 0)
                    y = 0;
                System.Windows.Point Pl_p = new System.Windows.Point(ploti, y);
                dataSource4.AppendAsync(base.Dispatcher, Pl_p);
                    ploti++;
            }
        }

        private void AnimatedPlot_center_Variation(object sender, EventArgs e)
        {
            if (transfer_value == true)
            {
                System.Windows.Point Pl_p = new System.Windows.Point(vari_ploti, variable_V);
                dataSource5.AppendAsync(base.Dispatcher, Pl_p);
            }
        }
        private void AnimatedPlot_hwratio_Variation(object sender, EventArgs e)
        {
            if (transfer_value == true)
            {
                System.Windows.Point Pl_p = new System.Windows.Point(vari_ploti, variable_R);
                dataSource6.AppendAsync(base.Dispatcher, Pl_p);
            }
            
        }
        private void AnimatedPlot_widthest_Variation(object sender, EventArgs e)
        {
            if (transfer_value == true)
            {
                System.Windows.Point Pl_p = new System.Windows.Point(vari_ploti, variable_W);
                dataSource7.AppendAsync(base.Dispatcher, Pl_p);
            }
        }
        private void AnimatedPlot_area_Variation(object sender, EventArgs e)
        {
            if (transfer_value == true)
            {
                System.Windows.Point Pl_p = new System.Windows.Point(vari_ploti, variable_A);
                dataSource8.AppendAsync(base.Dispatcher, Pl_p);

                vari_ploti++;
                variable_A = 0;
                variable_R = 0;
                variable_V = 0;
                variable_W = 0;
            }
        }
    }
}
