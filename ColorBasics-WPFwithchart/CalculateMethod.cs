using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Media.Media3D;
using Microsoft.Kinect;
using System.Drawing;
using System.Drawing.Imaging;
using System.Runtime.InteropServices;
using System.IO;


namespace Microsoft.Samples.Kinect.ColorBasics
{
    static class CalculateMethod 
    {
        public static int width = 640;
        public static int height = 480;
        public static Vector3D newO = new Vector3D();
        public static Vector3D newX = new Vector3D();
        public static Vector3D newY = new Vector3D();
        public static Vector3D newZ = new Vector3D();     
             
      
        public static Matrix3D Tmatrix;     
        public static Matrix3D Rmatrix;
             
        public static Vector3D TransPortVector(Vector3D realPoint) //單點做世界座標轉換
        {
            Vector3D result = CalculateMethod.MatrixMulVector(realPoint, Tmatrix);

            Vector3D after = new Vector3D();
            after.X = Rmatrix.M11 * result.X + Rmatrix.M12 * result.Y + Rmatrix.M13 * result.Z;
            after.Y = Rmatrix.M21 * result.X + Rmatrix.M22 * result.Y + Rmatrix.M23 * result.Z;
            after.Z = Rmatrix.M31 * result.X + Rmatrix.M32 * result.Y + Rmatrix.M33 * result.Z;

            return after;
        }

        public static Vector3D TransPortVector(SkeletonPoint skel) //單點做世界座標轉換
        {
            Vector3D realPoint = new Vector3D(skel.X, skel.Y, skel.Z);
            realPoint = TransPortVector(realPoint);

            return realPoint;
        }   

        public static Vector3D PlaneVector(Vector3D P1, Vector3D P2, Vector3D P3) //區塊平面法向量
        {
            Vector3D V1 = new Vector3D();
            Vector3D V2 = new Vector3D();
            Vector3D V3 = new Vector3D();

            V1 = P1 - P3;
            V2 = P2 - P3;
            V3 = Vector3D.CrossProduct(V1, V2);
            return V3;
        }

        public static Vector3D MatrixMulVector(Vector3D v, Matrix3D m)
        {
            Vector3D result = new Vector3D();
            result.X = m.M11 * v.X + m.M12 * v.Y + m.M13 * v.Z + m.M14 * 1;
            result.Y = m.M21 * v.X + m.M22 * v.Y + m.M23 * v.Z + m.M24 * 1;
            result.Z = m.M31 * v.X + m.M32 * v.Y + m.M33 * v.Z + m.M34 * 1;

            return result;
        }

        public static void CalcTMatrix(Vector3D value)
        {
            Tmatrix = new Matrix3D(1, 0, 0, -value.X,
                                   0, 1, 0, -value.Y,
                                   0, 0, 1, -value.Z,
                                   0, 0, 0, 1);    
        }     

        public static void PrintMatrix(Matrix3D R)
        {
            Console.WriteLine("rotation matrix 1 : ");
            Console.WriteLine(R.M11 + "," + R.M12 + "," + R.M13 + "," + R.M14);
            Console.WriteLine(R.M21 + "," + R.M22 + "," + R.M23 + "," + R.M24);
            Console.WriteLine(R.M31 + "," + R.M32 + "," + R.M33 + "," + R.M34);
            Console.WriteLine(R.OffsetX + "," + R.OffsetY + "," + R.OffsetZ + "," + R.M44);
        }           

        public static void ClearAll()
        {
            Tmatrix = new Matrix3D();          
            Rmatrix = new Matrix3D();

            newO = new Vector3D();
            newX = new Vector3D();
            newY = new Vector3D();
            newZ = new Vector3D();
        }

        public static void DefaultData()
        {
            StreamReader sr = new StreamReader("EnvirInfo_def.txt");
            while(!sr.EndOfStream)
            {
                string sline = sr.ReadLine();
                string[] split = sline.Split(',');
                if (split[0] == "floor")
                {
                    Floor.roadFunction = new Vector3D(Convert.ToDouble(split[1]), Convert.ToDouble(split[2]), Convert.ToDouble(split[3]));
                    Floor.d = Convert.ToDouble(split[4]);
                }
                else if (split[0] == "T matrix")
                {
                    Tmatrix = new Matrix3D(Convert.ToDouble(split[1]),Convert.ToDouble(split[2]),Convert.ToDouble(split[3]),Convert.ToDouble(split[4]),Convert.ToDouble(split[5]),Convert.ToDouble(split[6]),Convert.ToDouble(split[7]),Convert.ToDouble(split[8]),Convert.ToDouble(split[9]),Convert.ToDouble(split[10]),Convert.ToDouble(split[11]),Convert.ToDouble(split[12]),Convert.ToDouble(split[13]),Convert.ToDouble(split[14]),Convert.ToDouble(split[15]),Convert.ToDouble(split[16]));
                }
                else if (split[0] == "R matrix")
                {
                    Rmatrix = new Matrix3D(Convert.ToDouble(split[1]), Convert.ToDouble(split[2]), Convert.ToDouble(split[3]), Convert.ToDouble(split[4]), Convert.ToDouble(split[5]), Convert.ToDouble(split[6]), Convert.ToDouble(split[7]), Convert.ToDouble(split[8]), Convert.ToDouble(split[9]), Convert.ToDouble(split[10]), Convert.ToDouble(split[11]), Convert.ToDouble(split[12]), Convert.ToDouble(split[13]), Convert.ToDouble(split[14]), Convert.ToDouble(split[15]), Convert.ToDouble(split[16]));            
                }              
            }                   
           
        }
    }

    //偵測平面
    class DetectPlant : IDisposable
    {     
        public double[] plantParameter = new double[5];
        public double Ythreshold = 2.5;
        
        Vector3D[, ,] PlantPoint;
        Vector3D[,] PlantVector;
        Vector3D TempVector = new Vector3D();
        Vector3D RefPoint = new Vector3D();
        Vector3D PrePoint = new Vector3D();

        int[,] PlantMask;
        int Block_W, Block_H;
        int PlantRate;
        int GrowCount;
        bool RecordPlantFlag = false;
        byte Group_R, Group_G, Group_B;  //上物體顏色用
        double maxDis = Double.MinValue;
   
        const int PlantSize = 10; //Window Size 平面區塊大小       
        const int ERROR_IDX = 2;
        const int TAKE_IDX = 1;
        const int DETECT_IDX = 0;
        const double Distance = 0.08;
        const int Width = 640;
        const int Height = 480;                
  
        Random random = new Random();
        byte[] byte_PlantGroup = new byte[Width * Height * 4]; //生長路面圖;

        #region 版本一 : 一口氣將所有典轉為世界座標
        /*
        public void detect(Vector3D[,] RealWorldPoint, int Width, int Height, double rate)
        {           
            Block_W = Width / PlantSize; //區塊切割寬數量
            Block_H = Height / PlantSize; //區塊切割高數量
            PlantRate = (int)(rate * (Block_H * Block_W));

            PlantMask = new int[Block_H, Block_W]; //路面生長遮罩                     
            PlantPoint  = new Vector3D[Block_H, Block_W, 3];
            PlantVector = new Vector3D[Block_H, Block_W];
          
            #region  計算區塊的平面法向量
            for (int y = 0; y < Block_H - 1; y++)
            {
                for (int x = 0; x < Block_W - 1; x++)
                {
                    int PosIndex = y * Block_W + x;
                    Point[] point = new Point[3];
                    Vector3D tempPoint = new Vector3D();

                    point[0].X = x * PlantSize;
                    point[0].Y = y * PlantSize;
                    point[1].X = (x + 1) * PlantSize;
                    point[1].Y = y * PlantSize;
                    point[2].X = x * PlantSize;
                    point[2].Y = (y + 1) * PlantSize;

                    //篩選平面區塊
                    for (int i = 0; i < 3; i++)
                    {
                        //取得世界空間座標
                        tempPoint = RealWorldPoint[point[i].X, point[i].Y];
                        if (tempPoint.Z < 0.8 || tempPoint.Z > 4)  //取不到深度資訊時把遮罩設定為error
                        {
                            PlantMask[y, x] = ERROR_IDX;
                            RecordPlantFlag = false; //取消紀錄PlantVector[]
                            break;
                        }
                        else
                        {
                            RecordPlantFlag = true;
                            PlantPoint[y, x, i] = tempPoint;
                        }
                    }

                    //紀錄區塊平面法向量
                    if (RecordPlantFlag == true)
                    {
                        PlantVector[y, x] = CalculateMethod.PlaneVector(PlantPoint[y, x, 0], PlantPoint[y, x, 1], PlantPoint[y, x, 2]);

                        if (PlantVector[y, x].Y > 0 && PlantVector[y, x].Z < 0)
                        {
                            PlantVector[y, x].Normalize();
                        }
                        else
                        {
                            PlantMask[y, x] = ERROR_IDX;
                        }
                    }
                }
            }

            #endregion
    
            #region 計算路面方程式

            Vector3D PlantNormal = new Vector3D();
            Vector3D MinPoint = new Vector3D();
            Vector3D[,] RecordPlant = new Vector3D[Block_H * Block_W, 2];          
     
            double minDistance = 0;

            for (int y = 0; y < Block_H; y++)
                for (int x = 0; x < Block_W; x++)
                {

                    if (PlantMask[y, x] == DETECT_IDX)
                    {
                        GrowCount = 0;
                        TempVector = PlantVector[y, x];
                        RefPoint = PlantPoint[y, x, 0];
                        GrowPlant(x, y);

                        if (GrowCount >= PlantRate)
                        {
                            RefPoint = RefPoint / GrowCount;

                            double distance = Math.Abs(Vector3D.DotProduct(TempVector, RefPoint) / Math.Pow((TempVector.X * TempVector.X + TempVector.Y * TempVector.Y + TempVector.Z * TempVector.Z), 0.5));

                            if (distance > minDistance) //找最低平面
                            {
                                minDistance = distance;
                                MinPoint = RefPoint;
                                PlantNormal = TempVector;
                                PlantNormal.Normalize();  //應該可以省略
                            }

                        }
                        else
                        {
                            PlantMask[y, x] = ERROR_IDX;
                        }

                    }
                }            

            double plantParameter_a, plantParameter_b, plantParameter_c, plantParameter_d, plantParameter_ABC;  //aX+bY+cZ = d   
      
            plantParameter_a = PlantNormal.X;
            plantParameter_b = PlantNormal.Y;
            plantParameter_c = PlantNormal.Z;
            plantParameter_d = Vector3D.DotProduct(PlantNormal, MinPoint);
            plantParameter_ABC = Math.Pow(Math.Pow(PlantNormal.X, 2) + Math.Pow(PlantNormal.Y, 2) + Math.Pow(PlantNormal.Z, 2), 0.5); //到平面距離公式(分母)
            PlantNormal.Normalize();

            plantParameter[0] = plantParameter_a;
            plantParameter[1] = plantParameter_b;
            plantParameter[2] = plantParameter_c;
            plantParameter[3] = plantParameter_d;
            plantParameter[4] = plantParameter_ABC;

            Console.WriteLine(plantParameter[0] + "," + plantParameter[1] + "," + plantParameter[2] + ".  d = " + plantParameter[3]);

            #endregion

        }
        
        //畫地面
        public void paint(Vector3D[,] RealWorldPoint, ColorImagePoint[] _mappedDepthLocations, ref byte[] draw_Floor)
        {
            int Width  = Block_W * PlantSize;
            int Height = Block_H * PlantSize;

            for (int y = 0; y < 480; y++)
                for (int x = 0; x < 640; x++)
                {
                    Vector3D RealPos = RealWorldPoint[x, y];
                    double d = (plantParameter[0] * RealPos.X + plantParameter[1] * RealPos.Y + plantParameter[2] * RealPos.Z - plantParameter[3]) / plantParameter[4];

                    if (d < 0)    d = 0 - d;
                    if (d < Distance && RealPos.Z > 0.8 && RealPos.Z < 4)
                    {
                        ColorImagePoint point = _mappedDepthLocations[y * 640 + x];
                        if ((point.X >= 0 && point.X < 640) && (point.Y >= 0 && point.Y < 480))
                        {
                            draw_Floor[(point.Y * Width + point.X) * 4] = (byte)(0);
                            draw_Floor[(point.Y * Width + point.X) * 4 + 1] = (byte)(255);
                            draw_Floor[(point.Y * Width + point.X) * 4 + 2] = (byte)(255);
                        }
                    }
                }
        }
        */
        #endregion

        #region 版本二 : 只轉需要用的點到世界座標

        //全部偵測
        public List<Object3D> Detect(double rate, SkeletonPoint[] skeleton, byte[] colorPixel)
        {
            Block_W = Width / PlantSize;  //區塊切割寬數量
            Block_H = Height / PlantSize; //區塊切割高數量
            PlantRate = (int)(rate * (Block_H * Block_W));

            PlantMask = new int[Block_H, Block_W]; //路面生長遮罩                     
            PlantPoint = new Vector3D[Block_H, Block_W, 3];
            PlantVector = new Vector3D[Block_H, Block_W];

            List<Object3D> plants = new List<Object3D>();

            #region  計算區塊的平面法向量
            for (int y = 0; y < Block_H - 1; y++)
            {
                for (int x = 0; x < Block_W - 1; x++)
                {
                    //int PosIndex = y * Block_W + x;
                    Point[] point = new Point[3];
                    Vector3D tempPoint = new Vector3D();

                    point[0].X = x * PlantSize;
                    point[0].Y = y * PlantSize;
                    point[1].X = (x + 1) * PlantSize;
                    point[1].Y = y * PlantSize;
                    point[2].X = x * PlantSize;
                    point[2].Y = (y + 1) * PlantSize;

                    //篩選平面區塊
                    for (int i = 0; i < 3; i++)
                    {
                        //取得世界空間座標
                        tempPoint = CalculateMethod.TransPortVector(skeleton[point[i].X + point[i].Y * Width]);
                        if (tempPoint.Y == Ythreshold) //Vector3D.Equals(tempPoint, new Vector3D(0, 0, 0)) || 
                        {
                            PlantMask[y, x] = ERROR_IDX;
                            RecordPlantFlag = false; //取消紀錄PlantVector[] '

                            byte_PlantGroup[4 * (x + y * Width) + 3] = (byte)0;
                            for (int _i = 0; _i < PlantSize; _i++)
                                for (int _j = 0; _j < PlantSize; _j++)
                                {
                                    byte_PlantGroup[((point[i].Y + _i) * Width * 4) + (point[i].X + _j) * 4 + 3] = (byte)0;
                                }

                            break;
                        }
                        else
                        {
                            RecordPlantFlag = true;
                            PlantPoint[y, x, i] = tempPoint;
                        }
                    }

                    //紀錄區塊平面法向量
                    if (RecordPlantFlag == true)
                    {
                        tempPoint = CalculateMethod.TransPortVector(skeleton[((x + 1) * PlantSize) + ((y + 1) * PlantSize) * Width]);
                        Vector3D tempVector = CalculateMethod.PlaneVector(tempPoint, PlantPoint[y, x, 1], PlantPoint[y, x, 2]);

                        PlantVector[y, x] = CalculateMethod.PlaneVector(PlantPoint[y, x, 0], PlantPoint[y, x, 1], PlantPoint[y, x, 2]);

                        //統一方向
                        if (PlantVector[y, x].Y < 0) PlantVector[y, x] = PlantVector[y, x] * -1;

                        if (tempVector.Y < 0) tempVector = tempPoint * -1;

                        //PlantVector[y, x] += tempVector;

                        PlantVector[y, x].Normalize();                        
                    }
                }
            }

            #endregion

            #region 計算路面方程式
            
            Vector3D PlantNormal = new Vector3D();
            Vector3D MinPoint = new Vector3D();
            Vector3D[,] RecordPlant = new Vector3D[Block_H * Block_W, 2];
            Object3D obj = new Object3D();

            double minDistance = 5;
            int id = 0;

            for (int y = 0; y < Block_H; y++)
                for (int x = 0; x < Block_W; x++)
                {
                    if (PlantMask[y, x] == DETECT_IDX)
                    {
                        Group_R = (byte)random.Next(100, 255);
                        Group_G = (byte)random.Next(100, 255);
                        Group_B = (byte)random.Next(100, 255);
                        obj = new Object3D();

                        maxDis = Double.MinValue;
                        GrowCount = 0;
                        TempVector = PlantVector[y, x];
                        RefPoint = PlantPoint[y, x, 0];
                        PrePoint = PlantPoint[y, x, 0];
                        GrowPlant(x, y, ref obj, skeleton, colorPixel);

                        if (GrowCount >= PlantRate)  
                        {
                            Console.WriteLine("plant id " + id + "," + maxDis);

                            RefPoint = RefPoint / GrowCount;

                            //因為皆為平面,故法向量相差不大,直接取其平均點的Y值作為高度判斷,取最低點
                            double distance = RefPoint.Y;
                            if (distance < minDistance) //找最低平面
                            {
                                minDistance = distance;
                                MinPoint = RefPoint;   //嚴格講起來是取平均點
                                PlantNormal = TempVector;
                                PlantNormal.Normalize();  //應該可以省略
                            }

                            if (obj.points.Count != 0)
                            {
                                obj.max_plant.normal_vector = TempVector;
                                obj.max_plant.normal_vector.Normalize();
                                obj.max_plant.d = Vector3D.DotProduct(obj.max_plant.normal_vector, RefPoint);
                                obj.max_plant.height = Math.Abs(RefPoint.Y);
                                obj.max_plant.draw_range[0] = new System.Windows.Point(obj.points.Min(p => p.X), obj.points.Min(p => p.Y));
                                obj.max_plant.draw_range[1] = new System.Windows.Point(obj.points.Max(p => p.X), obj.points.Max(p => p.Y));
                                
                                //取得平面的範圍
                                obj.GetRange(Ythreshold, true);
                                obj.max_plant.plant_range[0] = new Point3D(obj.cube_range[0].X, RefPoint.Y, obj.cube_range[0].Z);
                                obj.max_plant.plant_range[1] = new Point3D(obj.cube_range[1].X, RefPoint.Y, obj.cube_range[1].Z);
                                obj.cube_prange[0] = obj.max_plant.draw_range[0];
                                obj.cube_prange[1] = obj.max_plant.draw_range[1];
                                obj.ID = id;

                             
                                for (int i = 0; i < 256; i++)
                                {                                   
                                    obj.xhistrogram[i] /= obj.points.Count;                               
                                }

                                plants.Add(obj);

                                id++;
                            }
                        }
                        else
                        {
                            PlantMask[y, x] = ERROR_IDX;
                            byte_PlantGroup[4 * (x + y * Width) + 3] = (byte)0;

                            for (int i = 0; i < PlantSize; i++)
                                for (int j = 0; j < PlantSize; j++)
                                {
                                    byte_PlantGroup[((y * PlantSize + i) * Width * 4) + (x * PlantSize + j) * 4 + 3] = (byte)255;
                                }
                        }
                    }
                }
            
            double plantParameter_a, plantParameter_b, plantParameter_c, plantParameter_d, plantParameter_ABC;  //aX+bY+cZ = d   

            plantParameter_a = PlantNormal.X;
            plantParameter_b = PlantNormal.Y;
            plantParameter_c = PlantNormal.Z;
            plantParameter_d = Vector3D.DotProduct(PlantNormal, MinPoint);
            plantParameter_ABC = Math.Pow(Math.Pow(PlantNormal.X, 2) + Math.Pow(PlantNormal.Y, 2) + Math.Pow(PlantNormal.Z, 2), 0.5); //到平面距離公式(分母)
            //PlantNormal.Normalize();

            plantParameter[0] = plantParameter_a;
            plantParameter[1] = plantParameter_b;
            plantParameter[2] = plantParameter_c;
            plantParameter[3] = plantParameter_d;
            plantParameter[4] = plantParameter_ABC;

            Floor.roadFunction = new Vector3D(plantParameter_a, plantParameter_b, plantParameter_c);
            Floor.d = plantParameter_d;

            return plants;

            #endregion
        }

        public Object3D Detect(int SX, int SY, int EX, int EY, double rate, Vector3D[] realpoints, double goal_height)
        {
            Block_W = (EX - SX) / PlantSize; //區塊切割寬數量
            Block_H = (EY - SY) / PlantSize; //區塊切割高數量
            PlantRate = (int)(rate * (Block_H * Block_W));

            PlantMask = new int[Block_H, Block_W]; //路面生長遮罩                     
            PlantPoint = new Vector3D[Block_H, Block_W, 3];
            PlantVector = new Vector3D[Block_H, Block_W];

            List<Object3D> objects = new List<Object3D>();

            #region  計算區塊的平面法向量

            for (int y = 0; y < Block_H - 1; y++)
            {
                for (int x = 0; x < Block_W - 1; x++)
                {
                    //int PosIndex = y * Block_W + (x + SX);
                    Point[] point = new Point[3];
                    Vector3D tempPoint = new Vector3D();

                    point[0].X = x * PlantSize + SX;
                    point[0].Y = y * PlantSize + SY;
                    point[1].X = (x + 1) * PlantSize + SX;
                    point[1].Y = y * PlantSize + SY;
                    point[2].X = x * PlantSize + SX;
                    point[2].Y = (y + 1) * PlantSize + SY;

                    //篩選平面區塊
                    for (int i = 0; i < 3; i++)
                    {
                        //取得世界空間座標
                        tempPoint = realpoints[point[i].X + point[i].Y * Width];
                        if (Vector3D.Equals(tempPoint, new Vector3D(0, 0, 0)) || tempPoint.Y == Ythreshold)
                        {
                            PlantMask[y, x] = ERROR_IDX;
                            RecordPlantFlag = false; //取消紀錄PlantVector[]  
                            break;
                        }
                        else
                        {
                            RecordPlantFlag = true;
                            PlantPoint[y, x, i] = tempPoint;
                        }
                    }

                    //紀錄區塊平面法向量
                    if (RecordPlantFlag == true)
                    {
                        PlantVector[y, x] = CalculateMethod.PlaneVector(PlantPoint[y, x, 0], PlantPoint[y, x, 1], PlantPoint[y, x, 2]);

                        //統一方向
                        if (PlantVector[y, x].Y < 0) PlantVector[y, x] = PlantVector[y, x] * -1;
                        PlantVector[y, x].Normalize();
                    }
                }
            }

            #endregion

            #region 求出區域最大平面

            Vector3D[,] RecordPlant = new Vector3D[Block_H * Block_W, 2];
            List<Point> plan_points = new List<Point>();
            List<Point> relation = new List<Point>();

            double maxGrowCount = 0;
            int id = 0;
            int newid = 0;

            for (int y = 0; y < Block_H; y++)
                for (int x = 0; x < Block_W; x++)
                {
                    if (PlantMask[y, x] == DETECT_IDX)
                    {
                        Group_R = (byte)random.Next(0, 255);
                        Group_G = (byte)random.Next(0, 255);
                        Group_B = (byte)random.Next(0, 255);
                        plan_points.Clear();

                        GrowCount = 0;
                        TempVector = PlantVector[y, x];
                        RefPoint = PlantPoint[y, x, 0];
                        GrowPlant(x, y, ref plan_points, SX, SY);

                        if (GrowCount >= PlantRate)
                        {
                            RefPoint = RefPoint / GrowCount;
                            Vector3D test = TempVector;
                            test.Normalize();


                            if (plan_points.Count != 0)
                            {
                                //找最大平面 // 找高度最接近的平面
                                //if (goal_height != -1)
                                //{
                                //    //if (GrowCount > maxGrowCount)
                                //    if (Math.Abs(Math.Abs(RefPoint.Y) - goal_height) < maxGrowCount)
                                //    {
                                //        //maxGrowCount = GrowCount;
                                //        maxGrowCount = Math.Abs(Math.Abs(RefPoint.Y) - goal_height);
                                //        newid = id;
                                //    }
                                //}
                                //else
                                double AngleIJ = Vector3D.AngleBetween(test, Floor.roadFunction);
                         
                                {
                                    if (GrowCount > maxGrowCount && Math.Abs(RefPoint.Y) > 0.1 &&　AngleIJ < 10) // 
                                    {
                                        maxGrowCount = GrowCount;
                                        newid = id;
                                    }
                                }

                                relation.Add(new Point(GrowCount, id));

                                //求出範圍
                                double minx = Double.MaxValue, maxx = Double.MinValue, minz = Double.MaxValue, maxz = Double.MinValue;
                                foreach (Point p in plan_points)
                                {
                                    Vector3D tmp = realpoints[p.X + p.Y * Width];
                                    if (tmp.Y != Ythreshold)
                                    {
                                        if (tmp.X < minx) minx = tmp.X;
                                        if (tmp.X > maxx) maxx = tmp.X;
                                        if (tmp.Z < minz) minz = tmp.Z;
                                        if (tmp.Z > maxz) maxz = tmp.Z;
                                    }
                                }

                                //求出物體資訊
                                Object3D object3d = new Object3D();
                                object3d.max_plant.normal_vector = TempVector;
                                object3d.max_plant.normal_vector.Normalize();
                                object3d.max_plant.d = Vector3D.DotProduct(object3d.max_plant.normal_vector, RefPoint);
                                object3d.max_plant.draw_range[0] = new System.Windows.Point(plan_points.Min(p => p.X), plan_points.Min(p => p.Y));
                                object3d.max_plant.draw_range[1] = new System.Windows.Point(plan_points.Max(p => p.X), plan_points.Max(p => p.Y));
                                object3d.max_plant.center = new Vector3D((object3d.cube_range[0].X + object3d.cube_range[1].X) / 2, RefPoint.Y, (object3d.cube_range[1].Z + object3d.cube_range[0].Z) / 2);
                                object3d.max_plant.height = Math.Abs(RefPoint.Y);
                                object3d.cube_range[0] = object3d.max_plant.plant_range[0] = new Point3D(minx, RefPoint.Y, minz);
                                object3d.cube_range[1] = object3d.max_plant.plant_range[1] = new Point3D(maxx, RefPoint.Y, maxz);
                                object3d.ID = id;
                                objects.Add(object3d);

                                id++;
                            }
                        }
                        else
                        {
                            PlantMask[y, x] = ERROR_IDX;
                        }
                    }
                }

            #endregion
            
            //relation.Sort(Tools.ComparePointX);
            //relation.ToArray();
            //if (goal_height != -1)
            //{
            //    for (int i = 0; i < relation.Count; i++)
            //    {
            //        Object3D tmp = objects.Find(p => p.ID == relation[i].Y);
            //        if (Math.Abs(tmp.max_plant.height - goal_height) < 0.1)
            //        {
            //            newid = relation[i].Y;
            //            break;
            //        }
            //    }
            //}
            //else
            //{
            //    for (int i = 0; i < relation.Count; i++)
            //    {
            //        Object3D tmp = objects.Find(p => p.ID == relation[i].Y);
            //        if (Math.Abs(tmp.max_plant.height - 0.1) >= 0.1)
            //        {
            //            newid = relation[i].Y;
            //            break;
            //        }
            //    }
            //}

            Object3D newobject = objects.Find(p => p.ID == newid);
            if(newobject != null)
                Console.WriteLine("max grow count " + maxGrowCount + ",  " + Vector3D.AngleBetween(newobject.max_plant.normal_vector, Floor.roadFunction));

            return newobject;
        }
            

        //上色
        public void paint(ref byte[] draw_Floor, SkeletonPoint[] skeleton)
        {
            int Width = Block_W * PlantSize;
            int Height = Block_H * PlantSize;

            for (int y = 0; y < 480; y++)
                for (int x = 0; x < 640; x++)
                {
                    Vector3D RealPos = CalculateMethod.TransPortVector(skeleton[x + y * Width]); 
                    double d = (plantParameter[0] * RealPos.X + plantParameter[1] * RealPos.Y + plantParameter[2] * RealPos.Z - plantParameter[3]) / plantParameter[4];

                    if (d < 0) d = 0 - d;
                    if (d < Distance) 
                    {
                        draw_Floor[(y * 640 + x) * 4] = (byte)(0);
                        draw_Floor[(y * 640 + x) * 4 + 1] = (byte)(255);
                        draw_Floor[(y * 640 + x) * 4 + 2] = (byte)(255);                        
                    }
                }
        }

        //上色
        public void paintPlans(ref byte[] draw_Floor)
        {        
            int Width = Block_W * PlantSize;
            int Height = Block_H * PlantSize;

            for (int y = 0; y < 480; y++)
                for (int x = 0; x < 640; x++)
                {                   
                    if (byte_PlantGroup[4 * (x + y * Width)] != 0)
                    {
                        draw_Floor[4 * (x + y * Width)] = (byte)byte_PlantGroup[4 * (x + y * Width)];
                        draw_Floor[4 * (x + y * Width) + 1] = (byte)byte_PlantGroup[4 * (x + y * Width) + 1];
                        draw_Floor[4 * (x + y * Width) + 2] = (byte)byte_PlantGroup[4 * (x + y * Width) + 2];                      
                    }                  
                }

        }    
        
        #endregion


        private void GrowPlant(int x, int y, ref Object3D obj, SkeletonPoint[] skeleton, byte[] colorPixel)
        {
            Vector3D temp = TempVector;
            temp.Normalize();
            double DP = Vector3D.DotProduct(temp, PlantVector[y, x]); //內積 限制角度範圍 30度0.86 45度 0.7 
            double Dis = Tools.Distance(PrePoint, PlantPoint[y, x, 0]);            

            if ((x > 0 && x < Block_W - 1) && (y > 0 && y < Block_H - 1) && (DP > 0.86) && Dis <= 0.3)// 
            {             
                obj.points.Add(new System.Windows.Point(x * PlantSize, y * PlantSize));
                obj.points3d.Add(PlantPoint[y, x, 0]);

                PlantMask[y, x] = TAKE_IDX;
                RefPoint = RefPoint + PlantPoint[y, x, 0];
                TempVector = TempVector + PlantVector[y, x];
                PrePoint = PlantPoint[y, x, 0];
                GrowCount++; //計算平面數量          

                if (Dis > maxDis) maxDis = Dis; //測試

                for (int i = 0; i < PlantSize; i++)
                    for (int j = 0; j < PlantSize; j++)
                    {
                        byte_PlantGroup[((y * PlantSize + i) * Width * 4) + (x * PlantSize + j) * 4 + 2] = (byte)Group_R;
                        byte_PlantGroup[((y * PlantSize + i) * Width * 4) + (x * PlantSize + j) * 4 + 1] = (byte)Group_G;
                        byte_PlantGroup[((y * PlantSize + i) * Width * 4) + (x * PlantSize + j) * 4 + 0] = (byte)Group_B;

                        //加上高度值踢掉物體部分 , 避免抓到物體高度
                        Vector3D tmp = CalculateMethod.TransPortVector(skeleton[(int)(x * PlantSize + j) + (int)(y * PlantSize + i) * Width]);
                        double _Dis = Tools.Distance(PrePoint, tmp);   

                        if (tmp.Y != Ythreshold && tmp.Y >= 0.08 && _Dis <= 0.3)
                        {
                            obj.points.Add(new System.Windows.Point(x * PlantSize + j, y * PlantSize + i));
                            obj.points3d.Add(tmp);
                        }

                        int B = colorPixel[(int)((y * PlantSize + i) * Width + x * PlantSize + j) * 4 + 0];
                        int G = colorPixel[(int)((y * PlantSize + i) * Width + x * PlantSize + j) * 4 + 1];
                        int R = colorPixel[(int)((y * PlantSize + i) * Width + x * PlantSize + j) * 4 + 2];

                        double U = (-0.169 * R - 0.331 * G + 0.500 * B + 128); //量化
                        double Y = 0.299 * R + 0.587 * G + 0.114 * B;
                        obj.xhistrogram[(int)U]++;                    
                    }

                if (PlantMask[y, (x - 1)] == DETECT_IDX) //left
                {
                    GrowPlant(x - 1, y, ref obj, skeleton, colorPixel);
                }

                if (PlantMask[(y - 1), x] == DETECT_IDX) //up
                {
                    GrowPlant(x, y - 1, ref obj, skeleton, colorPixel);
                }
                if (PlantMask[(y + 1), x] == DETECT_IDX) //down
                {
                    GrowPlant(x, y + 1, ref obj, skeleton, colorPixel);
                }
                if (PlantMask[y, (x + 1)] == DETECT_IDX) //right
                {
                    GrowPlant(x + 1, y, ref obj, skeleton, colorPixel);
                }

                //if (PlantMask[(y - 1), (x - 1)] == DETECT_IDX) //left
                //{
                //    GrowPlant(x - 1, y - 1, ref obj, skeleton, colorPixel);
                //}
                //if (PlantMask[(y - 1), (x + 1)] == DETECT_IDX) //up
                //{
                //    GrowPlant((x + 1), y - 1, ref obj, skeleton, colorPixel);
                //}
                //if (PlantMask[(y + 1), (x - 1)] == DETECT_IDX) //down
                //{
                //    GrowPlant(x - 1, y + 1, ref obj, skeleton, colorPixel);
                //}
                //if (PlantMask[(y + 1), (x + 1)] == DETECT_IDX) //right
                //{
                //    GrowPlant(x + 1, y + 1, ref obj, skeleton, colorPixel);
                //}
            }
        }

        private void GrowPlant(int x, int y, ref List<Point> points, int SX, int SY)
        {
            Vector3D temp = TempVector;
            temp.Normalize();
            double DP = Vector3D.DotProduct(temp, PlantVector[y, x]); //內積 限制角度範圍 30度0.86 45度 0.7 
          
            if ((x > 0 && x < Block_W - 1) && (y > 0 && y < Block_H - 1) && (DP > 0.86))
            {
                points.Add(new Point(x * PlantSize + SX, y * PlantSize + SY));

                if (x * PlantSize + SX > 640 || y * PlantSize + SY > 480)
                    Console.WriteLine("overfload " + (x * PlantSize + SX) + "," + (y * PlantSize + SY));


                PlantMask[y, x] = TAKE_IDX;
                RefPoint = RefPoint + PlantPoint[y, x, 0];
                TempVector = TempVector + PlantVector[y, x];
                GrowCount++; //計算平面數量                

                for (int i = 0; i < PlantSize; i++)
                    for (int j = 0; j < PlantSize; j++)
                    {
                        byte_PlantGroup[((y * PlantSize + i + SY) * 640 * 4) + (x * PlantSize + j + SX) * 4 + 2] = (byte)Group_R;
                        byte_PlantGroup[((y * PlantSize + i + SY) * 640 * 4) + (x * PlantSize + j + SX) * 4 + 1] = (byte)Group_G;
                        byte_PlantGroup[((y * PlantSize + i + SY) * 640 * 4) + (x * PlantSize + j + SX) * 4 + 0] = (byte)Group_B;
                        points.Add(new Point(x * PlantSize + j + SX, y * PlantSize + i + SY));

                        if (x * PlantSize + SX > 640 || y * PlantSize + SY > 480)
                            Console.WriteLine("overfload " + (x * PlantSize + j + SX) + "," + (y * PlantSize + i + SY));
                    }

                if (PlantMask[y, (x - 1)] == DETECT_IDX) //left
                {
                    GrowPlant(x - 1, y, ref points, SX, SY);
                }
                if (PlantMask[(y - 1), x] == DETECT_IDX) //up
                {
                    GrowPlant(x, y - 1, ref points, SX, SY);
                }
                if (PlantMask[(y + 1), x] == DETECT_IDX) //down
                {
                    GrowPlant(x, y + 1, ref points, SX, SY);
                }
                if (PlantMask[y, (x + 1)] == DETECT_IDX) //right
                {
                    GrowPlant(x + 1, y, ref points, SX, SY);
                }
            }
        }
               
        ~DetectPlant()
        {
            DoDispose();
        }
        
        public void Dispose()
        {
            DoDispose();
        }

        private void DoDispose()
        {
            //Console.WriteLine(" DetectPlant Dispose !");
            PlantPoint = null;
            PlantVector = null;
            PlantMask = null;
        }

    }

    //顏色 & 深度成長
    public class RegionGrow
    {
        public int W = 640;
        public int H = 480;
        public byte[] colorPixels;
        public SkeletonPoint[] skeleton;
        public short[,] depth;
        public double Ythreshold;
        public double ColorThreshold;
        public byte[] objectMask;
        public int[] detectmarker;

        public double avgDis = 0;
        public double maxDis = 0;
        public double bigavgDis = 0;
        public int cdis = 0;
        public int ctotal = 0;
        public Vector3D maxdisPoint;
              
        private int pR, pG, pB;
        private short pdepth;
        private Vector3D p3dpoint;      
        private double cSY;   

        const int DETECTED = 1;
        const int UNDETECT = 0;
        const int ERROR = 2;
   
        public List<Vector3D> newpoints;

        public void Initial(byte[] color, SkeletonPoint[] skel, short[,] dep, double Y, byte[] floorimage)
        {
            colorPixels = new byte [W * H * 4];
            objectMask  = new byte[W * H * 4];
            skeleton    = new SkeletonPoint[W * H * 4];
            depth       = new short[W, H];
            detectmarker = new int[W * H];
            newpoints = new List<Vector3D>();
            p3dpoint = new Vector3D();          
            ColorThreshold = 30;

            Array.Copy(color,colorPixels,color.Length);
            Array.Copy(skel,skeleton,skel.Length);
            Array.Copy(floorimage, objectMask, floorimage.Length);
            depth = dep;
            Ythreshold = Y;         
        }          

        public List<Vector3D> Detect(Object3D obj, List<Object3D> objects)
        {            
            for (int o = 0; o < objects.Count; o++)
            {
                if (objects[o].ID != obj.ID)
                {                   
                    for (int i = 0; i < objects[o].points.Count; i++)
                    {
                        double x = objects[o].points[i].X;
                        double y = objects[o].points[i].Y;
                        detectmarker[(int)(x + y * W)] = ERROR;
                    }
                }
            }

            avgDis = 0;
            maxDis = 0;
            cdis = 0;
            ctotal = 0;
            bigavgDis = 0;
            maxdisPoint = new Vector3D();

            int SX = (int)(obj.max_plant.draw_range[0].X + obj.max_plant.draw_range[1].X) / 2;
            int SY = (int)(obj.max_plant.draw_range[0].Y + obj.max_plant.draw_range[1].Y) / 2;

            pB = colorPixels[4 * (int)(SX + SY * W) + 0];
            pG = colorPixels[4 * (int)(SX + SY * W) + 1];
            pR = colorPixels[4 * (int)(SX + SY * W) + 2];
            pdepth = depth[SX, SY];
            p3dpoint = CalculateMethod.TransPortVector(skeleton[(int)(SX + SY * W)]);         
            cSY = 0.299 * pR + 0.587 * pG + 0.114 * pB;

            double dx = obj.max_plant.draw_range[1].X - obj.max_plant.draw_range[0].X;
            double dy = obj.max_plant.draw_range[1].Y - obj.max_plant.draw_range[0].Y;
      
            try
            {
                if (pdepth != Ythreshold)
                    GrowColor(SX, SY);
                else Console.WriteLine("start error");

                Console.WriteLine("avgDis  = " + (double)avgDis / cdis + ",  maxdis " + maxDis + "   count " + cdis + "  ctotal " + ctotal + ", bigavg " + (double)bigavgDis / (ctotal - cdis));
                //Console.WriteLine("max point " + maxdisPoint);
                avgDis = 0;
                maxDis = 0;
                cdis = 0;
                ctotal = 0;
                bigavgDis = 0;
                maxdisPoint = new Vector3D();
            }
            catch (StackOverflowException error)
            {
                Console.WriteLine(error);
            }          
            return newpoints;
        }

        private int GrowColor(int x, int y)
        {         
            if (detectmarker[x + y * W] == UNDETECT)
            {
                int B = colorPixels[4 * (int)(x + y * W) + 0];
                int G = colorPixels[4 * (int)(x + y * W) + 1];
                int R = colorPixels[4 * (int)(x + y * W) + 2];
                short newdepth = depth[x, y];
                Vector3D tmp = CalculateMethod.TransPortVector(skeleton[(int)(x + y * W)]);

                double U  = (-0.169 * R  - 0.331 * G  + 0.500 * B  + 128); //量化
                double pU = (-0.169 * pR - 0.331 * pG + 0.500 * pB + 128); //量化

                double Y  = 0.299 * R  + 0.587 * G  + 0.114 * B;
                double pY = 0.299 * pR + 0.587 * pG + 0.114 * pB;
               
                double Dis = Tools.Distance(tmp, p3dpoint);
                ctotal++;    
                if (Dis > maxDis)
                {
                    maxDis = Dis;
                    maxdisPoint = tmp;
                }
                if (Dis > 0.5)
                {
                    bigavgDis += Dis;
                }
                else
                {
                    cdis++;
                    avgDis += Dis;
                }
                            
                if ((Math.Abs(R - pR) < ColorThreshold && Math.Abs(G - pG) < ColorThreshold && Math.Abs(B - pB) < ColorThreshold) && Math.Abs(Y - cSY) < 50 && (tmp.Y != Ythreshold && Dis <= 0.3)) // || (Math.Abs(R - pR) + Math.Abs(G - pG) + Math.Abs(B - pB)) < 50 &&
                {
                    newpoints.Add(tmp);
                    detectmarker[x + y * W] = DETECTED;
                    pB = B;
                    pG = G;
                    pR = R;
                    pdepth = newdepth;
                    p3dpoint = tmp;                

                    for (int i = 0; i < 5; i++)
                        for (int j = 0; j < 5; j++)
                        {
                            objectMask[4 * ((x + i) + (y + j) * W) + 0] = (byte)B;
                            objectMask[4 * ((x + i) + (y + j) * W) + 1] = (byte)G;
                            objectMask[4 * ((x + i) + (y + j) * W) + 2] = (byte)R;                            
                        }

                    if (x > 0 && detectmarker[(x - 5) + y * W] == UNDETECT)
                        GrowColor(x - 5, y);
                    if (y > 0 && detectmarker[x + (y - 5) * W] == UNDETECT)
                        GrowColor(x, y - 5);
                    if (x < W - 1 && detectmarker[(x + 5) + y * W] == UNDETECT)
                        GrowColor(x + 5, y);
                    if (y < H - 1 && detectmarker[x + (y + 5) * W] == UNDETECT)
                        GrowColor(x, y + 5);
                }
                else
                {
                    detectmarker[x + y * W] = DETECTED;

                    if (!(Math.Abs(R - pR) < ColorThreshold && Math.Abs(G - pG) < ColorThreshold && Math.Abs(B - pB) < ColorThreshold && Math.Abs(Y - cSY) < 50))//後者好像可以拿掉
                    {
                        if (!(Math.Abs(R - pR) < ColorThreshold))
                        {
                            for (int i = 0; i < 5; i++)
                                for (int j = 0; j < 5; j++)
                                {
                                    objectMask[4 * ((x + i) + (y + j) * W) + 0] = (byte)0;
                                    objectMask[4 * ((x + i) + (y + j) * W) + 1] = (byte)0;
                                    objectMask[4 * ((x + i) + (y + j) * W) + 2] = (byte)255;
                                }
                        }
                        if (!(Math.Abs(G - pG) < ColorThreshold))
                        {
                            for (int i = 0; i < 5; i++)
                                for (int j = 0; j < 5; j++)
                                {
                                    objectMask[4 * ((x + i) + (y + j) * W) + 0] = (byte)0;
                                    objectMask[4 * ((x + i) + (y + j) * W) + 1] = (byte)255;
                                    objectMask[4 * ((x + i) + (y + j) * W) + 2] = (byte)0;
                                }
                        }
                        if (!(Math.Abs(B - pB) < ColorThreshold))
                        {
                            for (int i = 0; i < 5; i++)
                                for (int j = 0; j < 5; j++)
                                {
                                    objectMask[4 * ((x + i) + (y + j) * W) + 0] = (byte)255;
                                    objectMask[4 * ((x + i) + (y + j) * W) + 1] = (byte)0;
                                    objectMask[4 * ((x + i) + (y + j) * W) + 2] = (byte)0;
                                }
                        }
                        else if (Math.Abs(Y - cSY) >= 50)
                        {
                            for (int i = 0; i < 5; i++)
                                for (int j = 0; j < 5; j++)
                                {
                                    objectMask[4 * ((x + i) + (y + j) * W) + 0] = (byte)0;
                                    objectMask[4 * ((x + i) + (y + j) * W) + 1] = (byte)255;
                                    objectMask[4 * ((x + i) + (y + j) * W) + 2] = (byte)255;
                                }
                        }        
                    }
                    if (tmp.Y != Ythreshold && Dis > 0.3) //Math.Abs(pdepth - newdepth) < 800)
                    {
                        for (int i = 0; i < 5; i++)
                            for (int j = 0; j < 5; j++)
                            {
                                objectMask[4 * ((x + i) + (y + j) * W) + 0] = (byte)255;
                                objectMask[4 * ((x + i) + (y + j) * W) + 1] = (byte)0;
                                objectMask[4 * ((x + i) + (y + j) * W) + 2] = (byte)255;
                            }                                                       
                    }                                      
                    return 0;
                }
            }
            return 0;
        }
          
        public void InitialMarker(byte[] floor)
        {
            for (int i = 0; i < W; i++)
                for (int j = 0; j < H; j++)
                {
                    if (floor[4 * (i + j * W)] == 255)
                        detectmarker[(int)(i + j * W)] = UNDETECT;
                    else if (floor[4 * (i + j * W)] == 0)
                        detectmarker[(int)(i + j * W)] = ERROR;
                }
        }
            
     
    }
       
}
