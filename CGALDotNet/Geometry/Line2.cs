﻿using System;
using System.Collections.Generic;
using System.Text;

using CGALDotNetGeometry.Numerics;
using CGALDotNetGeometry.Shapes;

namespace CGALDotNet.Geometry
{
    /// <summary>
    /// 
    /// </summary>
    /// <typeparam name="K"></typeparam>
    public sealed class Line2<K> : Line2 where K : CGALKernel, new()
    {

        /// <summary>
        /// A horizontal line on the x axis.
        /// </summary>
        public readonly static Line2<K> Horizontal = new Line2<K>(Point2d.Zero, Point2d.UnitX);

        /// <summary>
        /// A vertical line on the y axis.
        /// </summary>
	    public readonly static Line2<K> Vertical = new Line2<K>(Point2d.Zero, Point2d.UnitY);

        /// <summary>
        /// 
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <param name="c"></param>
        public Line2(double a, double b, double c) : base(a, b, c, new K())
        {

        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="p1"></param>
        /// <param name="p2"></param>
        public Line2(Point2d p1, Point2d p2) : base(p1, p2, new K())
        {

        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="p"></param>
        /// <param name="v"></param>
        public Line2(Point2d p, Vector2d v) : base(p, v, new K())
        {

        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="ptr"></param>
        internal Line2(IntPtr ptr) : base(new K(), ptr)
        {

        }

        /// <summary>
        /// 
        /// </summary>
        public Line2<K> Opposite => new Line2<K>(Kernel.Line2_Opposite(Ptr));

        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        public override string ToString()
        {
            return string.Format("[Line2<{0}>: A={1}, B={2}, C={3}]",
                Kernel.Name, A, B, C);
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="point"></param>
        /// <returns></returns>
        public Line2<K> Perpendicular(Point2d point)
        {
            var ptr = Kernel.Line2_Perpendicular(Ptr, point);
            return new Line2<K>(ptr);
        }

        /// <summary>
        /// Translate the shape.
        /// </summary>
        /// <param name="translation">The amount to translate.</param>
        public void Translate(Point2d translation)
        {
            Kernel.Line2_Transform(Ptr, translation, 0, 1);
        }

        /// <summary>
        /// Rotate the shape.
        /// </summary>
        /// <param name="rotation">The amount to rotate.</param>
        public void Rotate(Degree rotation)
        {
            Kernel.Line2_Transform(Ptr, Point2d.Zero, rotation.radian, 1);
        }

        /// <summary>
        /// Scale the shape.
        /// </summary>
        /// <param name="scale">The amount to scale.</param>
        public void Scale(double scale)
        {
            Kernel.Line2_Transform(Ptr, Point2d.Zero, 0, scale);
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="translation"></param>
        /// <param name="rotation"></param>
        /// <param name="scale"></param>
        public void Transform(Point2d translation, Degree rotation, double scale)
        {
            Kernel.Line2_Transform(Ptr, translation, rotation.radian, scale);    
        }
    }

    /// <summary>
    /// 
    /// </summary>
    public abstract class Line2 : CGALObject
    {
        /// <summary>
        /// 
        /// </summary>
        private Line2()
        {

        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <param name="c"></param>
        /// <param name="kernel"></param>
        internal Line2(double a, double b, double c, CGALKernel kernel)
        {
            Kernel = kernel.GeometryKernel2;
            Ptr = Kernel.Line2_Create(a, b, c);
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="p1"></param>
        /// <param name="p2"></param>
        /// <param name="kernel"></param>
        internal Line2(Point2d p1, Point2d p2, CGALKernel kernel)
        {
            Kernel = kernel.GeometryKernel2;
            Ptr = Kernel.CreateFromPoints(p1, p2);
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="p"></param>
        /// <param name="v"></param>
        /// <param name="kernel"></param>
        internal Line2(Point2d p, Vector2d v, CGALKernel kernel)
        {
            Kernel = kernel.GeometryKernel2;
            Ptr = Kernel.CreateFromPointVector(p, v);
        }

        internal Line2(CGALKernel kernel, IntPtr ptr) : base(ptr)
        {
            Kernel = kernel.GeometryKernel2;
        }

        /// <summary>
        /// 
        /// </summary>
        protected private GeometryKernel2 Kernel { get; private set; }

        /// <summary>
        /// Convert to shape struct.
        /// </summary>
        public Line2d Shape => new Line2d(A, B, C);

        /// <summary>
        /// 
        /// </summary>
        public double A
        {
            get { return Kernel.Line2_GetA(Ptr); }
            set { Kernel.Line2_SetA(Ptr, value); }
        }

        /// <summary>
        /// 
        /// </summary>
        public double B
        {
            get { return Kernel.Line2_GetB(Ptr); }
            set { Kernel.Line2_SetB(Ptr, value); }
        }

        /// <summary>
        /// 
        /// </summary>
        public double C
        {
            get { return Kernel.Line2_GetC(Ptr); }
            set { Kernel.Line2_SetC(Ptr, value); }
        }

        /// <summary>
        /// 
        /// </summary>
        public bool IsDegenerate => Kernel.Line2_IsDegenerate(Ptr);

        /// <summary>
        /// 
        /// </summary>
        public bool IsHorizontal => Kernel.Line2_IsHorizontal(Ptr);

        /// <summary>
        /// 
        /// </summary>
        public bool IsVertical => Kernel.Line2_IsVertical(Ptr); 

        /// <summary>
        /// 
        /// </summary>
        public Vector2d Vector => Kernel.Line2_Vector(Ptr);

        /// <summary>
        /// 
        /// </summary>
        /// <param name="point"></param>
        /// <returns></returns>
        public bool HasOn(Point2d point)
        {
            return Kernel.Line2_HasOn(Ptr, point);
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="point"></param>
        /// <returns></returns>
        public bool HasOnNegativeSide(Point2d point)
        {
            return Kernel.Line2_HasOnNegativeSide(Ptr, point);  
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="point"></param>
        /// <returns></returns>
        public bool HasOnPositiveSide(Point2d point)
        {
            return Kernel.Line2_HasOnPositiveSide(Ptr, point);  
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="y"></param>
        /// <returns></returns>
        public double X_On_Y(double y)
        {
            return Kernel.Line2_X_On_Y(Ptr, y);
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="x"></param>
        /// <returns></returns>
        public double Y_On_X(double x)
        {
            return Kernel.Line2_Y_On_X(Ptr, x);
        }

        /// <summary>
        /// 
        /// </summary>
        protected override void ReleasePtr()
        {
            Kernel.Line2_Release(Ptr);
        }

        /// <summary>
        /// Round the shape.
        /// </summary>
        /// <param name="digits">The number of digits to round to.</param>
        public void Round(int digits)
        {
            this.A = Math.Round(A, digits);
            this.B = Math.Round(B, digits);
            this.C = Math.Round(C, digits);
        }
    }
}
