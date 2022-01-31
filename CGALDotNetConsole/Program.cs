﻿using System;
using System.Collections.Generic;
using BenchmarkDotNet.Attributes;
using BenchmarkDotNet.Running;

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

namespace CGALDotNetConsole
{
    public class Program
    {
        

        public static void Main(string[] args)
        {

            var pmesh = PolyhedronFactory<EEK>.CreateCube();
            var smesh = SurfaceMeshFactory<EEK>.CreateCube(1, true);

            smesh.Print();
            smesh.PrintIndices(true, false, false, false, false);

            Console.WriteLine("");
            Console.WriteLine("Before " + smesh.VertexDegree(0));
    
            smesh.RemoveVertex(0);
            smesh.CollectGarbage();

            Console.WriteLine("After " + smesh.VertexDegree(0));
            Console.WriteLine("");

            smesh.Print();
            smesh.PrintIndices(true, false, false, false, false);
        }

    }
}
