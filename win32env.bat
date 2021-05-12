set ODMBASE=%~dp0
set GDALBASE=%ODMBASE%venv\Lib\site-packages\osgeo
set OSFMBASE=%ODMBASE%SuperBuild\install\bin\opensfm\bin

set PATH=%GDALBASE%;%ODMBASE%SuperBuild\install\bin;%OSFMBASE%
set PROJ_LIB=%GDALBASE%\data\proj
call venv\Scripts\activate