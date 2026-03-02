/******************************************************************************
 * Copyright 2017 RoboSense All rights reserved.
 * Suteng Innovation Technology Co., Ltd. www.robosense.ai

 * This software is provided to you directly by RoboSense and might
 * only be used to access RoboSense LiDAR. Any compilation,
 * modification, exploration, reproduction and redistribution are
 * restricted without RoboSense's prior consent.

 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ROBOSENSE BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
#ifndef REFERENCE_COMMON_BASIC_OPS_WRITE_PCD_H
#define REFERENCE_COMMON_BASIC_OPS_WRITE_PCD_H

#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/low_level_io.h>
#include <pcl/io/lzf.h>

namespace rally {

template<typename PointT>
std::string
generatePCDHeader(const pcl::PointCloud<PointT> &cloud, const int nr_points) {
    std::ostringstream oss;
    oss.imbue(std::locale::classic());

    oss << "# .PCD v0.7 - Point Cloud Data file format"
           "\nVERSION 0.7"
           "\nFIELDS";

    const auto fields = pcl::getFields<PointT>();

    std::stringstream field_names, field_types, field_sizes, field_counts;
    for (const auto &field: fields) {
        if (field.name == "_")
            continue;
        // Add the regular dimension
        field_names << " " << field.name;
        field_sizes << " " << pcl::getFieldSize(field.datatype);
        if ("rgb" == field.name)
            field_types << " " << "U";
        else
            field_types << " " << pcl::getFieldType(field.datatype);
        int count = std::abs(static_cast<int> (field.count));
        if (count == 0) count = 1;  // check for 0 counts (coming from older converter code)
        field_counts << " " << count;
    }
    oss << field_names.str();
    oss << "\nSIZE" << field_sizes.str()
        << "\nTYPE" << field_types.str()
        << "\nCOUNT" << field_counts.str();
    // If the user passes in a number of points value, use that instead
//    if (nr_points != std::numeric_limits<int>::max ())
//        oss << "\nWIDTH " << nr_points << "\nHEIGHT " << 1 << "\n";
//    else
    oss << "\nWIDTH " << cloud.width << "\nHEIGHT " << cloud.height << "\n";

    oss << "VIEWPOINT " << cloud.sensor_origin_[0] << " " << cloud.sensor_origin_[1] << " " << cloud.sensor_origin_[2]
        << " " <<
        cloud.sensor_orientation_.w() << " " <<
        cloud.sensor_orientation_.x() << " " <<
        cloud.sensor_orientation_.y() << " " <<
        cloud.sensor_orientation_.z() << "\n";

    // If the user passes in a number of points value, use that instead
//    if (nr_points != std::numeric_limits<int>::max ())
//        oss << "POINTS " << nr_points << "\n";
//    else
    oss << "POINTS " << cloud.size() << "\n";

    return (oss.str());
}

template<typename PointT>
int writePCDBinary(const std::string &file_name, const pcl::PointCloud<PointT> &cloud) {
    if (cloud.empty()) {
        PCL_WARN ("[writePCDBinary] Input point cloud has no data!\n");
    }
    int data_idx = 0;
    std::ostringstream oss;
    oss << generatePCDHeader<PointT>(cloud, 0) << "DATA binary\n";
    oss.flush();
    data_idx = static_cast<int> (oss.tellp());


    int fd = pcl::io::raw_open(file_name.c_str(), O_RDWR | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
    if (fd < 0) {
        throw pcl::IOException("[pcl::PCDWriter::writeBinary] Error during open!");
        return (-1);
    }
    // Mandatory lock file
//    boost::interprocess::file_lock file_lock;
//    setLockingPermissions (file_name, file_lock);

    auto fields = pcl::getFields<PointT>();
    std::vector<int> fields_sizes;
    std::size_t fsize = 0;
    std::size_t data_size = 0;
    std::size_t nri = 0;
    // Compute the total size of the fields
    for (const auto &field: fields) {
        if (field.name == "_")
            continue;

        int fs = field.count * pcl::getFieldSize(field.datatype);
        fsize += fs;
        fields_sizes.push_back(fs);
        fields[nri++] = field;
    }
    fields.resize(nri);

    data_size = cloud.size() * fsize;

    // Allocate disk space for the entire file to prevent bus errors.
    const int allocate_res = pcl::io::raw_fallocate(fd, data_idx + data_size);
    if (allocate_res != 0) {
        pcl::io::raw_close(fd);
//        resetLockingPermissions (file_name, file_lock);
        PCL_ERROR ("[pcl::PCDWriter::writeBinary] raw_fallocate(length=%zu) returned %i. errno: %d strerror: %s\n",
                   data_idx + data_size, allocate_res, errno, strerror(errno));

        throw pcl::IOException("[pcl::PCDWriter::writeBinary] Error during raw_fallocate ()!");
        return (-1);
    }

    char *map = static_cast<char *> (::mmap(nullptr, data_idx + data_size, PROT_WRITE, MAP_SHARED, fd, 0));
    if (map == reinterpret_cast<char *> (-1)) //MAP_FAILED)
    {
        pcl::io::raw_close(fd);
//        resetLockingPermissions (file_name, file_lock);
        throw pcl::IOException("[pcl::PCDWriter::writeBinary] Error during mmap ()!");
        return (-1);
    }

    // Copy the header
    memcpy(&map[0], oss.str().c_str(), data_idx);

    // Copy the data
    char *out = &map[0] + data_idx;
    for (const auto &point: cloud) {
        int nrj = 0;
        for (const auto &field: fields) {
            memcpy(out, reinterpret_cast<const char *> (&point) + field.offset, fields_sizes[nrj]);
            out += fields_sizes[nrj++];
        }
    }

    if (::munmap(map, (data_idx + data_size)) == -1) {
        pcl::io::raw_close(fd);
//        resetLockingPermissions (file_name, file_lock);
        throw pcl::IOException("[pcl::PCDWriter::writeBinary] Error during munmap ()!");
        return (-1);
    }
    // Close file
    pcl::io::raw_close(fd);
//    resetLockingPermissions (file_name, file_lock);
    return (0);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT>
int writeBinaryCompressed(const std::string &file_name,
                          const pcl::PointCloud<PointT> &cloud) {
    if (cloud.points.empty()) {
        throw pcl::IOException("[pcl::PCDWriter::writeBinaryCompressed] Input point cloud has no data!");
        return (-1);
    }
    int data_idx = 0;
    std::ostringstream oss;
    oss << generatePCDHeader<PointT>(cloud, 0) << "DATA binary_compressed\n";
    oss.flush();
    data_idx = static_cast<int> (oss.tellp());

    int fd = pcl::io::raw_open(file_name.c_str(), O_RDWR | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
    if (fd < 0) {
        throw pcl::IOException("[pcl::PCDWriter::writeBinaryCompressed] Error during open!");
        return (-1);
    }

    // Mandatory lock file
//    boost::interprocess::file_lock file_lock;
//    setLockingPermissions(file_name, file_lock);

    std::vector<pcl::PCLPointField> fields;
    size_t fsize = 0;
    size_t data_size = 0;
    size_t nri = 0;
    pcl::getFields(cloud, fields);
    std::vector<int> fields_sizes(fields.size());
    // Compute the total size of the fields
    for (size_t i = 0; i < fields.size(); ++i) {
        if (fields[i].name == "_")
            continue;

        fields_sizes[nri] = fields[i].count * pcl::getFieldSize(fields[i].datatype);
        fsize += fields_sizes[nri];
        fields[nri] = fields[i];
        ++nri;
    }
    fields_sizes.resize(nri);
    fields.resize(nri);

    // Compute the size of data
    data_size = cloud.points.size() * fsize;

    //////////////////////////////////////////////////////////////////////
    // Empty array holding only the valid data
    // data_size = nr_points * point_size
    //           = nr_points * (sizeof_field_1 + sizeof_field_2 + ... sizeof_field_n)
    //           = sizeof_field_1 * nr_points + sizeof_field_2 * nr_points + ... sizeof_field_n * nr_points
    char *only_valid_data = static_cast<char *> (malloc(data_size));

    // Convert the XYZRGBXYZRGB structure to XXYYZZRGBRGB to aid compression. For
    // this, we need a vector of fields.size () (4 in this case), which points to
    // each individual plane:
    //   pters[0] = &only_valid_data[offset_of_plane_x];
    //   pters[1] = &only_valid_data[offset_of_plane_y];
    //   pters[2] = &only_valid_data[offset_of_plane_z];
    //   pters[3] = &only_valid_data[offset_of_plane_RGB];
    //
    std::vector<char *> pters(fields.size());
    int toff = 0;
    for (size_t i = 0; i < pters.size(); ++i) {
        pters[i] = &only_valid_data[toff];
        toff += fields_sizes[i] * static_cast<int> (cloud.points.size());
    }

    // Go over all the points, and copy the data in the appropriate places
    for (size_t i = 0; i < cloud.points.size(); ++i) {
        for (size_t j = 0; j < fields.size(); ++j) {
            memcpy(pters[j], reinterpret_cast<const char *> (&cloud.points[i]) + fields[j].offset, fields_sizes[j]);
            // Increment the pointer
            pters[j] += fields_sizes[j];
        }
    }

    char *temp_buf = static_cast<char *> (malloc(static_cast<size_t> (static_cast<float> (data_size) * 1.5f + 8.0f)));
    // Compress the valid data
    unsigned int compressed_size = pcl::lzfCompress(only_valid_data,
                                                    static_cast<uint32_t> (data_size),
                                                    &temp_buf[8],
                                                    static_cast<uint32_t> (static_cast<float>(data_size) * 1.5f));
    unsigned int compressed_final_size = 0;
    // Was the compression successful?
    if (compressed_size) {
        char *header = &temp_buf[0];
        memcpy(&header[0], &compressed_size, sizeof(unsigned int));
        memcpy(&header[4], &data_size, sizeof(unsigned int));
        data_size = compressed_size + 8;
        compressed_final_size = static_cast<uint32_t> (data_size) + data_idx;
    } else {
//        resetLockingPermissions(file_name, file_lock);
        throw pcl::IOException("[pcl::PCDWriter::writeBinaryCompressed] Error during compression!");
        return (-1);
    }

    // Allocate disk space for the entire file to prevent bus errors.
    const int allocate_res = pcl::io::raw_fallocate(fd, data_idx + data_size);
    if (allocate_res != 0) {
        pcl::io::raw_close(fd);
//        resetLockingPermissions (file_name, file_lock);
        PCL_ERROR ("[pcl::PCDWriter::writeBinaryCompressed] raw_fallocate(length=%zu) returned %i. errno: %d strerror: %s\n",
                   data_idx + data_size, allocate_res, errno, strerror(errno));

        throw pcl::IOException("[pcl::PCDWriter::writeBinaryCompressed] Error during raw_fallocate ()!");
        return (-1);
    }

    // Prepare the map
    char *map = static_cast<char *> (mmap(0, compressed_final_size, PROT_WRITE, MAP_SHARED, fd, 0));
    if (map == reinterpret_cast<char *> (-1)) //MAP_FAILED)
    {
        pcl::io::raw_close(fd);
//        resetLockingPermissions(file_name, file_lock);
        throw pcl::IOException("[pcl::PCDWriter::writeBinaryCompressed] Error during mmap ()!");
        return (-1);
    }

    // Copy the header
    memcpy(&map[0], oss.str().c_str(), data_idx);
    // Copy the compressed data
    memcpy(&map[data_idx], temp_buf, data_size);

    // Unmap the pages of memory
    if (munmap(map, (compressed_final_size)) == -1) {
        pcl::io::raw_close(fd);
//        resetLockingPermissions(file_name, file_lock);
        throw pcl::IOException("[pcl::PCDWriter::writeBinaryCompressed] Error during munmap ()!");
        return (-1);
    }
    pcl::io::raw_close(fd);
//    resetLockingPermissions(file_name, file_lock);

    free(only_valid_data);
    free(temp_buf);
    return (0);
}

}  // namespace rally

#endif  // REFERENCE_COMMON_BASIC_OPS_WRITE_PCD_H
