using System;
using System.Text;
using System.Collections.Generic;
using BenchmarkDotNet.Attributes;
using BenchmarkDotNet.Running;

using CGALDotNetGeometry.Numerics;
using CGALDotNetGeometry.Shapes;
using CGALDotNetGeometry.Extensions;

using CGALDotNet;
using CGALDotNet.Geometry;
using CGALDotNet.Polygons;
using CGALDotNet.Polylines;
using CGALDotNet.Triangulations;
using CGALDotNet.Arrangements;
using CGALDotNet.Polyhedra;
using CGALDotNet.Meshing;
using CGALDotNet.Hulls;
using CGALDotNet.Processing;
using CGALDotNet.Extensions;
using CGALDotNet.Collections;
using CGALDotNet.Eigen;
using CGALDotNetTest.Processing;

namespace CGALDotNetConsole
{
    public class Program
    {


        static void Main(string[] args)
        {
            var test = new MeshProcessingMeshingSurfaceMeshTest();

            var testMesh1 = SurfaceMeshFactory<EEK>.CreateCube(1, true);
            var testMesh2 = SurfaceMeshFactory<EEK>.CreateCube(1, true);


            //Create the mesh.
            var points = new Point3d[]
            {
                new Point3d(-1,-1,-1),
                new Point3d(1,-1,-1),
                new Point3d(1,1,-1),
                new Point3d(-1,1,-1),
                new Point3d(-1,-1,1),
                new Point3d(1,-1,1),
                new Point3d(1,1,1),
                new Point3d(-1,1,1),
            };
            var indices = new int[]
            {
                0, 1, 2,
                2, 3, 0,

                6, 5, 4,
                4, 7, 6

            };
            var mesh1 = new Polyhedron3<EIK>(points, indices);
            var mesh2 = new Polyhedron3<EIK>(points, indices);

            Polyhedron3<EIK> result;
            var instance = MeshProcessingOrientation<EIK>.Instance;
            new MeshProcessingBoolean<EIK>().Op(POLYHEDRA_BOOLEAN.UNION, mesh1, mesh2, out result);
            //Get a instance to the processing helperr class.
            //Use the same kernel type (ie EIK)

            //Perform the op you want.
            instance.ReverseFaceOrientations(mesh1);


            test.Extrude();
        }


    }

    public enum POLYHEDRA_BOOLEAN
    {
        UNION,
        INTERSECT,
        DIFFERENCE
    };

    /// <summary>
    /// 
    /// </summary>
    /// <typeparam name="K"></typeparam>
    public sealed class MeshProcessingBoolean<K> : MeshProcessingBoolean where K : CGALKernel, new()
    {
        /// <summary>
        /// 
        /// </summary>
        public static readonly MeshProcessingBoolean<K> Instance = new MeshProcessingBoolean<K>();

        /// <summary>
        /// 
        /// </summary>
        public MeshProcessingBoolean() : base(new K())
        {

        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="ptr"></param>
        internal MeshProcessingBoolean(IntPtr ptr) : base(new K(), ptr)
        {

        }

        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        public override string ToString()
        {
            return string.Format("[MeshProcessingBoolean<{0}>: ]", Kernel.Name);
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="op"></param>
        /// <param name="mesh1"></param>
        /// <param name="mesh2"></param>
        /// <param name="result"></param>
        /// <returns></returns>
        public bool Op(POLYHEDRA_BOOLEAN op, Polyhedron3<K> mesh1, Polyhedron3<K> mesh2, out Polyhedron3<K> result)
        {
            switch (op)
            {
                case POLYHEDRA_BOOLEAN.UNION:
                    return Union(mesh1, mesh2, out result);
                case POLYHEDRA_BOOLEAN.INTERSECT:
                    return Intersection(mesh1, mesh2, out result);
                case POLYHEDRA_BOOLEAN.DIFFERENCE:
                    return Difference(mesh1, mesh2, out result);
            }

            result = null;
            return false;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="op"></param>
        /// <param name="mesh1"></param>
        /// <param name="mesh2"></param>
        /// <param name="result"></param>
        /// <returns></returns>
        public bool Op(POLYHEDRA_BOOLEAN op, SurfaceMesh3<K> mesh1, SurfaceMesh3<K> mesh2, out SurfaceMesh3<K> result)
        {
            switch (op)
            {
                case POLYHEDRA_BOOLEAN.UNION:
                    return Union(mesh1, mesh2, out result);
                case POLYHEDRA_BOOLEAN.INTERSECT:
                    return Intersection(mesh1, mesh2, out result);
                case POLYHEDRA_BOOLEAN.DIFFERENCE:
                    return Difference(mesh1, mesh2, out result);
            }

            result = null;
            return false;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="mesh1"></param>
        /// <param name="mesh2"></param>
        /// <param name="result"></param>
        /// <returns></returns>
        public bool Union(Polyhedron3<K> mesh1, Polyhedron3<K> mesh2, out Polyhedron3<K> result)
        {
            CheckIsValidException(mesh1);
            CheckIsValidException(mesh2);

            if (Kernel.Union_PH(mesh1.Ptr, mesh2.Ptr, out IntPtr resultPtr))
            {
                result = new Polyhedron3<K>(resultPtr);
                return true;
            }
            else
            {
                result = null;
                return false;
            }
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="mesh1"></param>
        /// <param name="mesh2"></param>
        /// <param name="result"></param>
        /// <returns></returns>
        public bool Union(SurfaceMesh3<K> mesh1, SurfaceMesh3<K> mesh2, out SurfaceMesh3<K> result)
        {
            CheckIsValidException(mesh1);
            CheckIsValidException(mesh2);

            if (Kernel.Union_SM(mesh1.Ptr, mesh2.Ptr, out IntPtr resultPtr))
            {
                result = new SurfaceMesh3<K>(resultPtr);
                return true;
            }
            else
            {
                result = null;
                return false;
            }
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="mesh1"></param>
        /// <param name="mesh2"></param>
        /// <param name="result"></param>
        /// <returns></returns>
        public bool Difference(Polyhedron3<K> mesh1, Polyhedron3<K> mesh2, out Polyhedron3<K> result)
        {
            CheckIsValidException(mesh1);
            CheckIsValidException(mesh2);

            if (Kernel.Difference_PH(mesh1.Ptr, mesh2.Ptr, out IntPtr resultPtr))
            {
                result = new Polyhedron3<K>(resultPtr);
                return true;
            }
            else
            {
                result = null;
                return false;
            }
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="mesh1"></param>
        /// <param name="mesh2"></param>
        /// <param name="result"></param>
        /// <returns></returns>
        public bool Difference(SurfaceMesh3<K> mesh1, SurfaceMesh3<K> mesh2, out SurfaceMesh3<K> result)
        {
            CheckIsValidException(mesh1);
            CheckIsValidException(mesh2);

            if (Kernel.Difference_SM(mesh1.Ptr, mesh2.Ptr, out IntPtr resultPtr))
            {
                result = new SurfaceMesh3<K>(resultPtr);
                return true;
            }
            else
            {
                result = null;
                return false;
            }
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="mesh1"></param>
        /// <param name="mesh2"></param>
        /// <param name="result"></param>
        /// <returns></returns>
        public bool Intersection(Polyhedron3<K> mesh1, Polyhedron3<K> mesh2, out Polyhedron3<K> result)
        {
            CheckIsValidException(mesh1);
            CheckIsValidException(mesh2);

            if (Kernel.Intersection_PH(mesh1.Ptr, mesh2.Ptr, out IntPtr resultPtr))
            {
                result = new Polyhedron3<K>(resultPtr);
                return true;
            }
            else
            {
                result = null;
                return false;
            }
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="mesh1"></param>
        /// <param name="mesh2"></param>
        /// <param name="result"></param>
        /// <returns></returns>
        public bool Intersection(SurfaceMesh3<K> mesh1, SurfaceMesh3<K> mesh2, out SurfaceMesh3<K> result)
        {
            CheckIsValidException(mesh1);
            CheckIsValidException(mesh2);

            if (Kernel.Intersection_PH(mesh1.Ptr, mesh2.Ptr, out IntPtr resultPtr))
            {
                result = new SurfaceMesh3<K>(resultPtr);
                return true;
            }
            else
            {
                result = null;
                return false;
            }
        }
    }

    /// <summary>
    /// 
    /// </summary>
    public abstract class MeshProcessingBoolean : PolyhedraAlgorithm
    {
        /// <summary>
        /// 
        /// </summary>
        private MeshProcessingBoolean()
        {

        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="kernel"></param>
        internal MeshProcessingBoolean(CGALKernel kernel)
        {
            Kernel = kernel.MeshProcessingBooleanKernel;
            Ptr = Kernel.Create();
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="kernel"></param>
        /// <param name="ptr"></param>
        internal MeshProcessingBoolean(CGALKernel kernel, IntPtr ptr) : base(ptr)
        {
            Kernel = kernel.MeshProcessingBooleanKernel;
            Ptr = ptr;
        }

        /// <summary>
        /// 
        /// </summary>
        internal MeshProcessingBooleanKernel Kernel { get; private set; }

        /// <summary>
        /// Release any unmanaged resources.
        /// </summary>
        protected override void ReleasePtr()
        {
            Kernel.Release(Ptr);
        }
    }
}
