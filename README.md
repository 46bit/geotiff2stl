# geotiff2stl

**Produce 3D topography models from GeoTIFFs, with a focus on usability and CNC**

Note: Right now only GeoTIFFs on the British National Grid are supported.

This program aims to be a reliable way to produce models, working around issues I experienced with other plugins etc.
The goal is to create ready-to-manufacture STL models, with reasonable file sizes and pleasant features.

## How to use

I get terrain data from the UK Environment Agency's 2m LIDAR scans, which you can download here: https://environment.data.gov.uk/DefraDataDownload/?Mode=survey.

The suggested workflow is to create GeoTIFFs of the area you are interested in using QGIS, and then run the tool against them.

To find interesting areas to create models of, try adding the [Environment Agency's ESRI ArcGIS endpoint](https://environment.data.gov.uk/dataset/09ea3b37-df3a-4e8b-ac69-fb0842227b04) to QGIS, as well as OpenStreetMap's XYZ endpoint.

Once you have a GeoTIFF, you can either run this program by building it yourself (Rust toolchain required) or via the prebuilt Docker image. `geotiff2stl -h` gives a thorough overview of the available options.
