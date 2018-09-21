/*
* Software License Agreement (BSD License)
*
* Point Cloud Library (PCL) - www.pointclouds.org
* Copyright (c) 2012-, Open Perception, Inc.
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of the copyright holder(s) nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
*/

#include "modifiedPclFunctions.hpp"

int saveOBJFile(const std::string &file_name, const pcl::TextureMesh &tex_mesh, unsigned precision)
{
  if (tex_mesh.cloud.data.empty ())
  {
    PCL_ERROR ("[pcl::io::saveOBJFile] Input point cloud has no data!\n");
    return (-1);
  }

  std::ostringstream fs;
  fs.precision (precision);

  // Define material file
  std::string mtl_file_name = file_name.substr (0, file_name.find_last_of (".")) + ".mtl";
  // Strip path for "mtllib" command
  std::string mtl_file_name_nopath = mtl_file_name;
  //std::cout << mtl_file_name_nopath << std::endl;
  mtl_file_name_nopath.erase (0, mtl_file_name.find_last_of ('/') + 1);

  /* Write 3D information */
  // number of points
  int nr_points  = tex_mesh.cloud.width * tex_mesh.cloud.height;
  int point_size = tex_mesh.cloud.data.size () / nr_points;

  // mesh size
  int nr_meshes = tex_mesh.tex_polygons.size ();
  // number of faces for header
  int nr_faces = 0;
  for (int m = 0; m < nr_meshes; ++m)
    nr_faces += tex_mesh.tex_polygons[m].size ();

  // Write the header information
  fs << "####" << std::endl;
  fs << "# OBJ dataFile simple version. File name: " << file_name << std::endl;
  fs << "# Vertices: " << nr_points << std::endl;
  fs << "# Faces: " <<nr_faces << std::endl;
  fs << "# Material information:" << std::endl;
  fs << "mtllib " << mtl_file_name_nopath << std::endl;
  fs << "####" << std::endl;

  // Write vertex coordinates
  fs << "# Vertices" << std::endl;
  for (int i = 0; i < nr_points; ++i)
  {
    int xyz = 0;
    // "v" just be written one
    bool v_written = false;
    for (size_t d = 0; d < tex_mesh.cloud.fields.size (); ++d)
    {
      int count = tex_mesh.cloud.fields[d].count;
      if (count == 0)
        count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
      int c = 0;
      // adding vertex
      if ((tex_mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) /*sensor_msgs::PointField::FLOAT32)*/ && (
                tex_mesh.cloud.fields[d].name == "x" ||
                tex_mesh.cloud.fields[d].name == "y" ||
                tex_mesh.cloud.fields[d].name == "z"))
      {
        if (!v_written)
        {
            // write vertices beginning with v
            fs << "v ";
            v_written = true;
        }
        float value;
        memcpy (&value, &tex_mesh.cloud.data[i * point_size + tex_mesh.cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
        fs << value;
        if (++xyz == 3)
            break;
        fs << " ";
      }
    }
    if (xyz != 3)
    {
      PCL_ERROR ("[pcl::io::saveOBJFile] Input point cloud has no XYZ data!\n");
      return (-2);
    }
    fs << std::endl;
  }
  fs << "# "<< nr_points <<" vertices" << std::endl;

//  // Write vertex normals
//  for (int i = 0; i < nr_points; ++i)
//  {
//    int xyz = 0;
//    // "vn" just be written one
//    bool v_written = false;
//    for (size_t d = 0; d < tex_mesh.cloud.fields.size (); ++d)
//    {
//      int count = tex_mesh.cloud.fields[d].count;
//      if (count == 0)
//      count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
//      int c = 0;
//      // adding vertex
//      if ((tex_mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
//      tex_mesh.cloud.fields[d].name == "normal_x" ||
//      tex_mesh.cloud.fields[d].name == "normal_y" ||
//      tex_mesh.cloud.fields[d].name == "normal_z"))
//      {
//        if (!v_written)
//        {
//          // write vertices beginning with vn
//          fs << "vn ";
//          v_written = true;
//        }
//        float value;
//        memcpy (&value, &tex_mesh.cloud.data[i * point_size + tex_mesh.cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
//        fs << value;
//        if (++xyz == 3)
//          break;
//        fs << " ";
//      }
//    }
//    if (xyz != 3)
//    {
//    //PCL_ERROR ("[pcl::io::saveOBJFile] Input point cloud has no normals!\n");
//    //return (-2);
//    }
//    fs << std::endl;
//  }
  // Write vertex texture with "vt" (adding latter)

  for (int m = 0; m < nr_meshes; ++m)
  {
    if(tex_mesh.tex_coordinates.size() == 0)
      continue;

    //PCL_INFO ("%d vertex textures in submesh %d\n", tex_mesh.tex_coordinates[m].size (), m);
    fs << "# " << tex_mesh.tex_coordinates[m].size() << " vertex textures in submesh " << m <<  std::endl;
    for (size_t i = 0; i < tex_mesh.tex_coordinates[m].size (); ++i)
    {
      fs << "vt ";
      fs <<  tex_mesh.tex_coordinates[m][i][0] << " " << tex_mesh.tex_coordinates[m][i][1] << std::endl;
    }
  }

  int f_idx = 0;

  // int idx_vt =0;
  //PCL_INFO ("Writting faces...\n");
  for (int m = 0; m < nr_meshes; ++m)
  {
    if (m > 0)
      f_idx += tex_mesh.tex_polygons[m-1].size ();

    if(tex_mesh.tex_materials.size() !=0)
    {
      fs << "# The material will be used for mesh " << m << std::endl;
      //TODO pbl here with multi texture and unseen faces
      fs << "usemtl " <<  tex_mesh.tex_materials[m].tex_name << std::endl;
      fs << "# Faces" << std::endl;
    }
    for (size_t i = 0; i < tex_mesh.tex_polygons[m].size(); ++i)
    {
      // Write faces with "f"
      fs << "f";
      size_t j = 0;
      // There's one UV per vertex per face, i.e., the same vertex can have
      // different UV depending on the face.
      for (j = 0; j < tex_mesh.tex_polygons[m][i].vertices.size (); ++j)
      {
        unsigned int idx = tex_mesh.tex_polygons[m][i].vertices[j] + 1;
        fs << " " << idx
        << "/" << 3*(i+f_idx) +j+1;
        //<< "/" << idx; // vertex index in obj file format starting with 1
      }
      fs << std::endl;
    }
    //PCL_INFO ("%d faces in mesh %d \n", tex_mesh.tex_polygons[m].size () , m);
    fs << "# "<< tex_mesh.tex_polygons[m].size() << " faces in mesh " << m << std::endl;
  }
  fs << "# End of File";

  // Close obj file
  //PCL_INFO ("Closing obj file\n");
  std::ofstream ofs(file_name.c_str ());
  ofs << fs.str() << std::endl;
  ofs.close ();

  /* Write material defination for OBJ file*/
  // Open file
  //PCL_INFO ("Writing material files\n");
  //dont do it if no material to write
  if(tex_mesh.tex_materials.size() ==0)
    return (0);

  // Empty string stream
  fs.str("");

  //std::cout << "MTL file is located at_ " << mtl_file_name << std::endl;
  // default
  fs << "#" << std::endl;
  fs << "# Wavefront material file" << std::endl;
  fs << "#" << std::endl;
  for(int m = 0; m < nr_meshes; ++m)
  {
    fs << "newmtl " << tex_mesh.tex_materials[m].tex_name << std::endl;
    fs << "Ka "<< tex_mesh.tex_materials[m].tex_Ka.r << " " << tex_mesh.tex_materials[m].tex_Ka.g << " " << tex_mesh.tex_materials[m].tex_Ka.b << std::endl; // defines the ambient color of the material to be (r,g,b).
    fs << "Kd "<< tex_mesh.tex_materials[m].tex_Kd.r << " " << tex_mesh.tex_materials[m].tex_Kd.g << " " << tex_mesh.tex_materials[m].tex_Kd.b << std::endl; // defines the diffuse color of the material to be (r,g,b).
    fs << "Ks "<< tex_mesh.tex_materials[m].tex_Ks.r << " " << tex_mesh.tex_materials[m].tex_Ks.g << " " << tex_mesh.tex_materials[m].tex_Ks.b << std::endl; // defines the specular color of the material to be (r,g,b). This color shows up in highlights.
    fs << "d " << tex_mesh.tex_materials[m].tex_d << std::endl; // defines the transparency of the material to be alpha.
    fs << "Ns "<< tex_mesh.tex_materials[m].tex_Ns  << std::endl; // defines the shininess of the material to be s.
    fs << "illum "<< tex_mesh.tex_materials[m].tex_illum << std::endl; // denotes the illumination model used by the material.
    // illum = 1 indicates a flat material with no specular highlights, so the value of Ks is not used.
    // illum = 2 denotes the presence of specular highlights, and so a specification for Ks is required.
    fs << "map_Kd " << tex_mesh.tex_materials[m].tex_file << std::endl;
    fs << "###" << std::endl;
  }


  std::ofstream omfs(mtl_file_name.c_str ());
  omfs << fs.str() << std::endl;
  omfs.close ();

  return (0);
}

bool getPixelCoordinates(const pcl::PointXYZ &pt, const pcl::TextureMapping<pcl::PointXYZ>::Camera &cam, pcl::PointXY &UV_coordinates)
{
    if (pt.z > 0)
    {
        // compute image center and dimension
        double sizeX = cam.width;
        double sizeY = cam.height;
        double cx, cy;
        if (cam.center_w > 0)
            cx = cam.center_w;
        else
            cx = sizeX / 2.0;
        if (cam.center_h > 0)
            cy = cam.center_h;
        else
            cy = sizeY / 2.0;

        double focal_x, focal_y;
        if (cam.focal_length_w > 0)
            focal_x = cam.focal_length_w;
        else
            focal_x = cam.focal_length;
        if (cam.focal_length_h > 0)
            focal_y = cam.focal_length_h;
        else
            focal_y = cam.focal_length;

        // project point on camera's image plane
        UV_coordinates.x = static_cast<float> ((focal_x * (pt.x / pt.z) + cx)); //horizontal
        UV_coordinates.y = static_cast<float> ((focal_y * (pt.y / pt.z) + cy)); //vertical

        // point is visible!
        if (UV_coordinates.x >= 1.0 && UV_coordinates.x <= (sizeX - 1.0) && UV_coordinates.y >= 1.0 && UV_coordinates.y <= (sizeY - 1.0))
        {
            return (true); // point was visible by the camera
        }
    }

    // point is NOT visible by the camera
    UV_coordinates.x = -1.0f;
    UV_coordinates.y = -1.0f;
    return (false); // point was not visible by the camera
}

bool isFaceProjected (const pcl::TextureMapping<pcl::PointXYZ>::Camera &camera, const pcl::PointXYZ &p1, const pcl::PointXYZ &p2, const pcl::PointXYZ &p3, pcl::PointXY &proj1, pcl::PointXY &proj2, pcl::PointXY &proj3)
{
    return (getPixelCoordinates(p1, camera, proj1) && getPixelCoordinates(p2, camera, proj2) && getPixelCoordinates(p3, camera, proj3));
}

void getTriangleCircumscribedCircleCentroid( const pcl::PointXY &p1, const pcl::PointXY &p2, const pcl::PointXY &p3, pcl::PointXY &circumcenter, double &radius)
{
 // compute centroid's coordinates (translate back to original coordinates)
 circumcenter.x = static_cast<float> (p1.x + p2.x + p3.x ) / 3;
 circumcenter.y = static_cast<float> (p1.y + p2.y + p3.y ) / 3;
 double r1 = (circumcenter.x - p1.x) * (circumcenter.x - p1.x) + (circumcenter.y - p1.y) * (circumcenter.y - p1.y) ;
 double r2 = (circumcenter.x - p2.x) * (circumcenter.x - p2.x) + (circumcenter.y - p2.y) * (circumcenter.y - p2.y) ;
 double r3 = (circumcenter.x - p3.x) * (circumcenter.x - p3.x) + (circumcenter.y - p3.y) * (circumcenter.y - p3.y) ;

 // radius
 radius = std::sqrt( std::max( r1, std::max( r2, r3) )) ;
}

bool checkPointInsideTriangle(const pcl::PointXY &p1, const pcl::PointXY &p2, const pcl::PointXY &p3, const pcl::PointXY &pt)
{
    // Compute vectors
    Eigen::Vector2d v0, v1, v2;
    v0(0) = p3.x - p1.x; v0(1) = p3.y - p1.y; // v0= C - A
    v1(0) = p2.x - p1.x; v1(1) = p2.y - p1.y; // v1= B - A
    v2(0) = pt.x - p1.x; v2(1) = pt.y - p1.y; // v2= P - A

    // Compute dot products
    double dot00 = v0.dot(v0); // dot00 = dot(v0, v0)
    double dot01 = v0.dot(v1); // dot01 = dot(v0, v1)
    double dot02 = v0.dot(v2); // dot02 = dot(v0, v2)
    double dot11 = v1.dot(v1); // dot11 = dot(v1, v1)
    double dot12 = v1.dot(v2); // dot12 = dot(v1, v2)

    // Compute barycentric coordinates
    double invDenom = 1.0 / (dot00*dot11 - dot01*dot01);
    double u = (dot11*dot02 - dot01*dot12) * invDenom;
    double v = (dot00*dot12 - dot01*dot02) * invDenom;

    // Check if point is in triangle
    return ((u >= 0) && (v >= 0) && (u + v < 1));
}
