#include <cstddef>
#include <cstdint>
#include <iostream>
#include <o3r_uncompress_di.h>
#include <ifm3d/fg/byte_buffer.h>
#include <ifm3d/fg/distance_image_info.h>

namespace ifm3d
{
    template <typename T>
    void dbgOutput(const std::string &name, const std::uint32_t begin, const std::uint32_t end, const T buffer)
    {
        std::cout << "-> " << name << "[" << begin << ":" <<  end << "]: ";
        for (auto i=begin; i<end; i++)
        {
            std::cout << buffer[i] << ",";
        }
        std::cout << std::endl;
    }

    std::vector<std::uint16_t> readU16Vector(std::size_t idx, const std::vector<std::uint8_t> &data_buffer, const std::uint32_t npts)
    {
       auto chunk_size =
        ifm3d::mkval<uint32_t>(data_buffer.data() + idx + CHUNK_SIZE_INFO_OFFSET);
       auto data_offset =
          ifm3d::mkval<std::uint32_t>(data_buffer.data() + idx + HEADER_SIZE_INFO_OFFSET);
       auto header_version =
            ifm3d::mkval<std::uint32_t>(data_buffer.data() + idx + HEADER_VERSION_INFO_OFFSET);

       std::cout << "header version: " << header_version
                << " chunk size: " << chunk_size 
                << " data offset: " << data_offset
                << std::endl;

      std::vector<std::uint16_t> u16_buffer(npts);
      auto dataSize = sizeof(uint16_t);
      idx += data_offset;
        for (auto i = 0; i < npts; ++i)
      {
        u16_buffer[i] =
          ifm3d::mkval<uint16_t>(data_buffer.data() + idx);
        idx += dataSize;
      }
      return u16_buffer;
    }

         std::unique_ptr<DistanceImageInfo> CreateDistanceImageInfo(const std::vector<std::uint8_t> &data_buffer,
                                                                   const std::size_t didx, const std::size_t aidx,
                                                                   const std::uint32_t width, const std::uint32_t height)
        {
            // Index of Distance Image Info chunk
            auto distimageidx =
                ifm3d::get_chunk_index(data_buffer, ifm3d::image_chunk::O3R_DISTANCE_IMAGE_INFORMATION);

            if (distimageidx == INVALID_IDX)
            {
                return {};
            }

            // header size
            auto data_offset =
                ifm3d::mkval<std::uint32_t>(data_buffer.data() + distimageidx + HEADER_SIZE_INFO_OFFSET);

            // Version
            auto dist_info_version = 
                ifm3d::mkval<std::uint32_t>(data_buffer.data() + distimageidx + data_offset);
            data_offset += sizeof(std::uint32_t);
            // Distance Resolution 
            auto dist_resolution = ifm3d::mkval<float>(data_buffer.data() + distimageidx + data_offset);
            data_offset += sizeof(float);
            // Amplitude Resolution
            auto ampl_resolution = ifm3d::mkval<float>(data_buffer.data() + distimageidx + data_offset);
            data_offset += sizeof(float);
            // Ampl Normalization Factor Vector
            std::vector<float> amp_norm_factors(3);
            for (auto i=0; i< 3; i++)
            {
                amp_norm_factors[i] = ifm3d::mkval<float>(data_buffer.data() + distimageidx + data_offset);
                data_offset += sizeof(float);
            }
            // Extrinsic Optic to User Vector
            std::vector<float> extrinsic_optic_to_user(6);
            for (auto i=0; i< 6; i++)
            {
                extrinsic_optic_to_user[i] = ifm3d::mkval<float>(data_buffer.data() + distimageidx + data_offset);
                data_offset += sizeof(float);
            }
            // Intrinsic Calibration
            IntrinsicCalibration intrinsic_calibration;
            intrinsic_calibration.model_iD = ifm3d::mkval<std::uint32_t>(data_buffer.data() + distimageidx + data_offset);
            data_offset += sizeof(std::uint32_t);
            for (auto i=0; i< 32; i++)
            {
                intrinsic_calibration.model_parameters[i] = ifm3d::mkval<float>(data_buffer.data() + distimageidx + data_offset);
                data_offset += sizeof(float);
            }
            // Inverse Intrinsic Calibration
            IntrinsicCalibration inverse_intrinsic_calibration;
            inverse_intrinsic_calibration.model_iD = ifm3d::mkval<std::uint32_t>(data_buffer.data() + distimageidx + data_offset);
            data_offset += sizeof(std::uint32_t);
            for (auto i=0; i< 32; i++)
            {
                inverse_intrinsic_calibration.model_parameters[i] = ifm3d::mkval<float>(data_buffer.data() + distimageidx + data_offset);
                data_offset += sizeof(float);
            }

            std::cout << "O3R_DISTANCE_IMAGE_INFORMATION \n\t-Header size: " << data_offset
                      << "\n\t-Version: " << dist_info_version 
                      << "\n\t-DistanceResolution: " << dist_resolution
                      << "\n\t-AmplitudeResolution: " << ampl_resolution
                      << "\n\t-AmpNormalizationFactors: " << amp_norm_factors[0] << "," << amp_norm_factors[1] << "," << amp_norm_factors[2]
                      << "\n\t-Extrinsic Optic to User (transZ): " <<  extrinsic_optic_to_user[2]
                      << "\n\t-Intrinsic Calib: model_iD: " <<  intrinsic_calibration.model_iD
                      << "\n\t-Inverseintrinsic Calib: model_iD: " <<  inverse_intrinsic_calibration.model_iD
                      << std::endl;

            return std::make_unique<DistanceImageInfo>(dist_resolution, ampl_resolution, amp_norm_factors,
                                                       extrinsic_optic_to_user, intrinsic_calibration, inverse_intrinsic_calibration,
                                                       readU16Vector(didx, data_buffer, width*height),
                                                       readU16Vector(aidx, data_buffer, width*height),
                                                       width, height);
        }

        DistanceImageInfo::DistanceImageInfo(
          const float dist_res,
          const float ampl_res,
          const std::vector<float> &amp_norm_fctrs,
          const std::vector<float>  &extr_opt_to_usr,
          const IntrinsicCalibration &intr_calib,
          const IntrinsicCalibration &inv_intr_calib,
          const std::vector<std::uint16_t> &distance_buffer,
          const std::vector<std::uint16_t> &amplitude_buffer,
          const std::uint32_t w, const std::uint32_t h)
          : dist_resolution(dist_res), ampl_resolution(ampl_res),
            amp_norm_factors(amp_norm_fctrs),
            extrinsic_optic_to_user(extr_opt_to_usr),
            intrinsic_calibration(intr_calib),
            inverse_intrinsic_calibration(inv_intr_calib),
            u16_distance_buffer(distance_buffer),
            u16_amplitude_buffer(amplitude_buffer),
            width(w), height(h)
      {
      }

    std::vector<std::uint8_t> DistanceImageInfo::getXYZDVector()
        {
            auto npts = width * height;
            float dist[npts], x[npts], y[npts], z[npts];
            uint16_t u16Dist[npts];

             for (auto i=0; i<npts; ++i)
            {
                u16Dist[i] = u16_distance_buffer[i];
            }
            dbgOutput("unified Distance", 9770, 9800, u16Dist);

            if(xyzdFromDistance(dist, x, y, z, u16Dist, dist_resolution,
                         intrinsic_calibration.model_iD, intrinsic_calibration.model_parameters,
                         extrinsic_optic_to_user[static_cast<int>(extrinsic_param::TRANS_X)],
                         extrinsic_optic_to_user[static_cast<int>(extrinsic_param::TRANS_Y)],
                         extrinsic_optic_to_user[static_cast<int>(extrinsic_param::TRANS_Z)],
                         extrinsic_optic_to_user[static_cast<int>(extrinsic_param::ROT_X)],
                         extrinsic_optic_to_user[static_cast<int>(extrinsic_param::ROT_Y)],
                         extrinsic_optic_to_user[static_cast<int>(extrinsic_param::ROT_Z)],
                         width, height)
               != 0)
            {
                return {};
            }

            dbgOutput("X-Value", 9770, 9800, x);
            dbgOutput("Y-Value", 9770, 9800, y);
            dbgOutput("Z-Value", 9770, 9800, z);
            dbgOutput("Distance", 9770, 9800, dist);

            int ix,iy,iz,id;
            float temp_bytes[4*npts];
            for (ix=0;ix<npts;++ix)
            {
                temp_bytes[ix] = x[ix];
            }
            for (iy=0;iy<npts;++iy)
            {
                temp_bytes[ix+iy] = y[iy];
            }
            for (iz=0;iz<npts;++iz)
            {
                temp_bytes[ix+iy+iz] = z[iz];
            }
            for (id=0;id<npts;++id)
            {
                temp_bytes[ix+iy+iz+id] = dist[id];
            }
            std::vector<std::uint8_t> xyzd_bytes(4*npts*sizeof(float));
            std::memcpy(xyzd_bytes.data(), temp_bytes,4*npts*sizeof(float));
            return xyzd_bytes;
    }


    std::vector<std::uint8_t> DistanceImageInfo::getAmplitudeVector()
    {
        auto npts = width * height;
        float amplitude[npts];

        dbgOutput("unified Amplitude", 0, 20, u16_amplitude_buffer.data());
 
        if(convertAmplitude( amplitude,
                             u16_amplitude_buffer.data(),
                             ampl_resolution,
                             width, height)
           != 0)

        {
            return {};
        }

        dbgOutput("calculated Amplitude", 0, 20, amplitude);
       
        std::vector<std::uint8_t> ampl_bytes(npts*sizeof(float));
        std::memcpy(ampl_bytes.data(), amplitude, npts*sizeof(float));
        return ampl_bytes;
    }
} // end: namespace ifm3d

