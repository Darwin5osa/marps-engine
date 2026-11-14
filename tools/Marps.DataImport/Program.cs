using System;
using System.Globalization;
using System.IO;
using System.Text.Json;
using BitMiracle.LibTiff.Classic;

// Converts crop_1025.tif (float32 heights) and hazards_1025.tif (hazard 0-3) into simple binaries
// read by the engine (terrain.f32 / hazard.u8 / terrain.json).
//
// Usage: dotnet run --project tools/Marps.DataImport -- <data_folder> [output_folder]
// Default output = <data_folder>/processed

internal static class Program
{
    // Crop parameters: upper-left corner (-900, 2569800), side 2000 m.
    // The southwest corner (originY = 2569800 - 2000) is used because the engine works with z pointing north.
    private const double OriginX = -900.0;
    private const double SideM = 2000.0;
    private const double OriginY = 2569800.0 - SideM;

    private static int Main(string[] args)
    {
        if (args.Length < 1)
        {
            Console.Error.WriteLine("Usage: Marps.DataImport <data_folder> [output_folder]");
            return 1;
        }

        string input = Path.GetFullPath(args[0]);
        string output = Path.GetFullPath(args.Length > 1 ? args[1] : Path.Combine(input, "processed"));
        Directory.CreateDirectory(output);

        string heightsTif = Path.Combine(input, "crop_1025.tif");
        string hazardTif = Path.Combine(input, "hazards_1025.tif");

        var (width, height, heights) = ReadTiff(heightsTif);
        var (hw, hh, hazardValues) = ReadTiff(hazardTif);
        if (hw != width || hh != height)
        {
            Console.Error.WriteLine($"Grid sizes do not match: {width}x{height} vs {hw}x{hh}.");
            return 2;
        }
        if (width != height)
        {
            Console.Error.WriteLine("A square grid is expected.");
            return 2;
        }

        // Heights binary: float32 little-endian, row 0 = north (as the TIFF stores it).
        var heightsBytes = new byte[heights.Length * 4];
        Buffer.BlockCopy(heights, 0, heightsBytes, 0, heightsBytes.Length);
        File.WriteAllBytes(Path.Combine(output, "terrain.f32"), heightsBytes);

        // Hazard binary: 1 byte per node, valid values 0..3.
        var hazard = new byte[hazardValues.Length];
        int[] counts = new int[4];
        for (int i = 0; i < hazardValues.Length; i++)
        {
            int v = (int)Math.Round(hazardValues[i]);
            if (v < 0 || v > 3)
            {
                Console.Error.WriteLine($"Hazard value out of range at index {i}: {hazardValues[i]}");
                return 3;
            }
            hazard[i] = (byte)v;
            counts[v]++;
        }
        File.WriteAllBytes(Path.Combine(output, "hazard.u8"), hazard);

        float zMin = float.MaxValue, zMax = float.MinValue;
        foreach (float h in heights)
        {
            if (float.IsNaN(h)) continue;
            if (h < zMin) zMin = h;
            if (h > zMax) zMax = h;
        }

        var meta = new
        {
            width,
            height,
            originX = OriginX,
            originY = OriginY,
            sideM = SideM,
            zMin = (double)zMin,
            zMax = (double)zMax,
            projection = "EQUIRECTANGULAR MARS (standard parallel 40, central meridian 28.33E, R 3387887.658234 m)",
            heightsFile = "terrain.f32",
            hazardFile = "hazard.u8",
        };
        File.WriteAllText(Path.Combine(output, "terrain.json"),
            JsonSerializer.Serialize(meta, new JsonSerializerOptions { WriteIndented = true }));

        Console.WriteLine($"Grid {width}x{height}, side {SideM} m, cell {SideM / (width - 1):0.######} m");
        Console.WriteLine($"Heights: {zMin.ToString("0.000", CultureInfo.InvariantCulture)} to {zMax.ToString("0.000", CultureInfo.InvariantCulture)} m");
        Console.WriteLine($"Hazard: 0={counts[0]} 1={counts[1]} 2={counts[2]} 3={counts[3]}");
        Console.WriteLine($"Output: {output}");
        return 0;
    }

    /// <summary>Reads the first band of a TIFF as double values (float32, uint8, uint16 or int16).</summary>
    private static (int Width, int Height, float[] Values) ReadTiff(string path)
    {
        if (!File.Exists(path)) throw new FileNotFoundException("TIFF not found", path);

        using var tif = Tiff.Open(path, "r");
        if (tif == null) throw new InvalidDataException($"No se pudo abrir {path}");

        int width = tif.GetField(TiffTag.IMAGEWIDTH)[0].ToInt();
        int height = tif.GetField(TiffTag.IMAGELENGTH)[0].ToInt();
        int bits = FieldInt(tif, TiffTag.BITSPERSAMPLE, 8);
        int format = FieldInt(tif, TiffTag.SAMPLEFORMAT, 1); // 1 = uint, 2 = int, 3 = IEEE float
        int spp = FieldInt(tif, TiffTag.SAMPLESPERPIXEL, 1);
        if (spp != 1) throw new InvalidDataException($"{path}: expected a single band, found {spp}.");

        int stride = tif.ScanlineSize();
        var buf = new byte[stride];
        var values = new float[width * height];

        for (int row = 0; row < height; row++)
        {
            if (!tif.ReadScanline(buf, row))
                throw new InvalidDataException($"{path}: could not read row {row}.");

            for (int col = 0; col < width; col++)
            {
                float v;
                if (bits == 32 && format == 3) v = BitConverter.ToSingle(buf, col * 4);
                else if (bits == 16 && format == 1) v = BitConverter.ToUInt16(buf, col * 2);
                else if (bits == 16 && format == 2) v = BitConverter.ToInt16(buf, col * 2);
                else if (bits == 8 && format == 1) v = buf[col];
                else throw new NotSupportedException($"{path}: unsupported format ({bits} bits, type {format}).");
                values[row * width + col] = v;
            }
        }
        return (width, height, values);
    }

    private static int FieldInt(Tiff tif, TiffTag tag, int fallback)
    {
        FieldValue[] f = tif.GetField(tag);
        return f == null || f.Length == 0 ? fallback : f[0].ToInt();
    }
}
