﻿using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;

using CGALDotNetGeometry.Numerics;

using REAL = System.Single;
using POINT2 = CGALDotNetGeometry.Numerics.Point2f;
using POINT3 = CGALDotNetGeometry.Numerics.Point3f;
using MATRIX2 = CGALDotNetGeometry.Numerics.Matrix2x2f;

namespace CGALDotNetGeometry.Shapes
{
    [Serializable]
    [StructLayout(LayoutKind.Sequential)]
    public struct Box2f : IEquatable<Box2f>
    {

        public POINT2 Min;

        public POINT2 Max;

        public Box2f(REAL min, REAL max)
        {
            Min = new POINT2(min);
            Max = new POINT2(max);
        }

        public Box2f(REAL minX, REAL maxX, REAL minY, REAL maxY)
        {
            Min = new POINT2(minX, minY);
            Max = new POINT2(maxX, maxY);
        }

        public Box2f(POINT2 min, POINT2 max)
        {
            Min = min;
            Max = max;
        }

        public POINT2 Corner00
        {
            get { return Min; }
        }

        public POINT2 Corner10
        {
            get { return new POINT2(Max.x, Min.y); }
        }

        public POINT2 Corner11
        {
            get { return Max; }
        }

        public POINT2 Corner01
        {
            get { return new POINT2(Min.x, Max.y); }
        }

        public POINT2 Center
        {
            get { return (Min + Max) * 0.5f; }
        }

        public POINT2 Size
        {
            get { return new POINT2(Width, Height); }
        }

        public REAL Width
        {
            get { return Max.x - Min.x; }
        }

        public REAL Height
        {
            get { return Max.y - Min.y; }
        }

        public REAL Area
        {
            get { return (Max.x - Min.x) * (Max.y - Min.y); }
        }

        public Box2f Bounds => this;

        public static Box2f operator +(Box2f box, REAL s)
        {
            return new Box2f(box.Min + s, box.Max + s);
        }

        public static Box2f operator +(Box2f box, POINT2 v)
        {
            return new Box2f(box.Min + v, box.Max + v);
        }

        public static Box2f operator -(Box2f box, REAL s)
        {
            return new Box2f(box.Min - s, box.Max - s);
        }

        public static Box2f operator -(Box2f box, POINT2 v)
        {
            return new Box2f(box.Min - v, box.Max - v);
        }

        public static Box2f operator *(Box2f box, REAL s)
        {
            return new Box2f(box.Min * s, box.Max * s);
        }

        public static Box2f operator /(Box2f box, REAL s)
        {
            return new Box2f(box.Min / s, box.Max / s);
        }

        public static Box2f operator *(Box2f box, MATRIX2 m)
        {
            return new Box2f(m * box.Min, m * box.Max);
        }

        public static bool operator ==(Box2f b1, Box2f b2)
        {
            return b1.Min == b2.Min && b1.Max == b2.Max;
        }

        public static bool operator !=(Box2f b1, Box2f b2)
        {
            return b1.Min != b2.Min || b1.Max != b2.Max;
        }

        public override bool Equals(object obj)
        {
            if (!(obj is Box2f)) return false;
            Box2f box = (Box2f)obj;
            return this == box;
        }

        public bool Equals(Box2f box)
        {
            return this == box;
        }

        public override int GetHashCode()
        {
            unchecked
            {
                int hash = (int)2166136261;
                hash = (hash * 16777619) ^ Min.GetHashCode();
                hash = (hash * 16777619) ^ Max.GetHashCode();
                return hash;
            }
        }

        public override string ToString()
        {
            return string.Format("[Box2f: Min={0}, Max={1}, Width={2}, Height={3}]", Min, Max, Width, Height);
        }

        public void GetCorners(IList<POINT2> corners)
        {
            corners[0] = new POINT2(Min.x, Min.y);
            corners[1] = new POINT2(Max.x, Min.y);
            corners[2] = new POINT2(Max.x, Max.y);
            corners[3] = new POINT2(Min.x, Max.y);
        }

        public void GetCornersXZ(IList<POINT3> corners, REAL y = 0)
        {
            corners[0] = new POINT3(Min.x, y, Min.y);
            corners[1] = new POINT3(Max.x, y, Min.y);
            corners[2] = new POINT3(Max.x, y, Max.y);
            corners[3] = new POINT3(Min.x, y, Max.y);
        }

        /// <summary>
        /// Returns the bounding box containing this box and the given point.
        /// </summary>
        public static Box2f Enlarge(Box2f box, POINT2 p)
        {
            var b = new Box2f();
            b.Min.x = Math.Min(box.Min.x, p.x);
            b.Min.y = Math.Min(box.Min.y, p.y);
            b.Max.x = Math.Max(box.Max.x, p.x);
            b.Max.y = Math.Max(box.Max.y, p.y);
            return b;
        }

        /// <summary>
        /// Returns the bounding box containing this box and the given box.
        /// </summary>
        public static Box2f Enlarge(Box2f box0, Box2f box1)
        {
            var b = new Box2f();
            b.Min.x = Math.Min(box0.Min.x, box1.Min.x);
            b.Min.y = Math.Min(box0.Min.y, box1.Min.y);
            b.Max.x = Math.Max(box0.Max.x, box1.Max.x);
            b.Max.y = Math.Max(box0.Max.y, box1.Max.y);
            return b;
        }

        /// <summary>
        /// Return a new box expanded by the amount.
        /// </summary>
        /// <param name="box">The box to expand.</param>
        /// <param name="amount">The amount to expand.</param>
        /// <returns>The expanded box.</returns>
        public static Box2d Expand(Box2d box, REAL amount)
        {
            return new Box2d(box.Min - amount, box.Max + amount);
        }

        /// <summary>
        /// Returns true if this box intersects the other box.
        /// </summary>
        public bool Intersects(Box2f a)
        {
            if (Max.x < a.Min.x || Min.x > a.Max.x) return false;
            if (Max.y < a.Min.y || Min.y > a.Max.y) return false;
            return true;
        }

        /// <summary>
        /// Does the box contain the other box.
        /// </summary>
        public bool Contains(Box2f a)
        {
            if (a.Max.x > Max.x || a.Min.x < Min.x) return false;
            if (a.Max.y > Max.y || a.Min.y < Min.y) return false;
            return true;
        }

        /// <summary>
        /// Does the box contain the point.
        /// </summary>
        public bool Contains(POINT2 p)
        {
            if (p.x > Max.x || p.x < Min.x) return false;
            if (p.y > Max.y || p.y < Min.y) return false;
            return true;
        }

        /// <summary>
        /// Find the closest point to the box.
        /// If point inside box return point.
        /// </summary>
        public POINT2 Closest(POINT2 p)
        {
            POINT2 c;

            if (p.x < Min.x)
                c.x = Min.x;
            else if (p.x > Max.x)
                c.x = Max.x;
            else
                c.x = p.x;

            if (p.y < Min.y)
                c.y = Min.y;
            else if (p.y > Max.y)
                c.y = Max.y;
            else
                c.y = p.y;

            return c;
        }

        /// <summary>
        /// Return the signed distance to the point. 
        /// If point is outside box field is positive.
        /// If point is inside box field is negative.
        /// </summary>
        public REAL SignedDistance(POINT2 p)
        {
            POINT2 d = (p - Center).Absolute - Size * 0.5f;
            POINT2 max = POINT2.Max(d, 0);
            return max.Magnitude + Math.Min(Math.Max(d.x, d.y), 0.0f);
        }

        /// <summary>
        /// Round the boxes components.
        /// </summary>
        /// <param name="digits">The number of digits to round to.</param>
        public void Round(int digits)
        {
            Min.Round(digits);
            Max.Round(digits);
        }

        public static Box2f CalculateBounds(IList<POINT2> vertices)
        {
            POINT2 min = POINT2.PositiveInfinity;
            POINT2 max = POINT2.NegativeInfinity;

            int count = vertices.Count;
            for (int i = 0; i < count; i++)
            {
                var v = vertices[i];
                if (v.x < min.x) min.x = v.x;
                if (v.y < min.y) min.y = v.y;

                if (v.x > max.x) max.x = v.x;
                if (v.y > max.y) max.y = v.y;
            }

            return new Box2f(min, max);
        }

        public static Box2f CalculateBounds(POINT2 a, POINT2 b)
        {
            REAL xmin = Math.Min(a.x, b.x);
            REAL xmax = Math.Max(a.x, b.x);
            REAL ymin = Math.Min(a.y, b.y);
            REAL ymax = Math.Max(a.y, b.y);

            return new Box2f(xmin, xmax, ymin, ymax);
        }

        public static Box2f CalculateBounds(IList<Segment2f> segments)
        {
            POINT2 min = POINT2.PositiveInfinity;
            POINT2 max = POINT2.NegativeInfinity;

            int count = segments.Count;
            for (int i = 0; i < count; i++)
            {
                var seg = segments[i];

                if (seg.A.x < min.x) min.x = seg.A.x;
                if (seg.A.y < min.y) min.y = seg.A.y;
                if (seg.B.x < min.x) min.x = seg.B.x;
                if (seg.B.y < min.y) min.y = seg.B.y;

                if (seg.A.x > max.x) max.x = seg.A.x;
                if (seg.A.y > max.y) max.y = seg.A.y;
                if (seg.B.x > max.x) max.x = seg.B.x;
                if (seg.B.y > max.y) max.y = seg.B.y;
            }

            return new Box2f(min, max);
        }

    }

}
















