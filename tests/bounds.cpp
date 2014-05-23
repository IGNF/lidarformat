/***********************************************************************

This file is part of the LidarFormat project source files.

LidarFormat is an open source library for efficiently handling 3D point
clouds with a variable number of attributes at runtime.


Homepage:

    http://code.google.com/p/lidarformat

Copyright:

    Institut Geographique National & CEMAGREF (2009)

Author:

    Adrien Chauve

Contributors:

        Nicolas David, Olivier Tournaire, Bruno Vallet



    LidarFormat is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published
    by the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    LidarFormat is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with LidarFormat.  If not, see <http://www.gnu.org/licenses/>.

***********************************************************************/

/** \example bounds.cpp by Bruno Vallet
 * This computes the bounds of a lidarformat file and writes them in the header
 */
#include <time.h>

#include "config_data_test.h"

#include "LidarFormat/LidarDataContainer.h"
#include "LidarFormat/LidarFile.h"
#include "LidarFormat/geometry/LidarCenteringTransfo.h"
#include "LidarFormat/tools/AttributeBounds.h"
#include "LidarFormat/file_formats/Ply2Lf.h"

using namespace Lidar;
using namespace std;

int main(int argc, char** argv)
{
    if(argc<2)
    {
        cout << "Usage: " << argv[0] << " lidarfile.xml" << std::endl;
        return 0;
    }
    for(int i=1; i<argc; i++)
    {
        time_t timer = clock();
        LidarFile file(argv[i]);
        cout << file.getMetaData();
        LidarDataContainer ldc;
        file.loadData(ldc);
        LidarCenteringTransfo transfo;
        file.loadTransfo(transfo);

        ldc.recomputeBounds(true);

        //LidarFile::save(ldc, argv[i], transfo);
        SavePly(ldc, transfo, argv[i]);
        std::cout << "Time: " << ( double ) timer/CLOCKS_PER_SEC << " s" << std::endl;
    }
    return 0;
}



