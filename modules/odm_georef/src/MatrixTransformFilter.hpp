#pragma once

#include <pcl/common/eigen.h>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/io/PlyReader.hpp>
#include <pdal/io/LasWriter.hpp>
#include <pdal/Options.hpp>
#include <pdal/Filter.hpp>

namespace pdal{
    template <typename Scalar>
    class MatrixTransformFilter : public Filter{
        Eigen::Transform<Scalar, 3, Eigen::Affine> transform;

        public:
            MatrixTransformFilter(const Eigen::Transform<Scalar, 3, Eigen::Affine> &transform)
                : transform(transform){};

            std::string getName() const { return "MatrixTransformFilter"; }

            virtual void filter(PointView &view)
            {
                for (PointId id = 0; id < view.size(); ++id)
                {
                    Scalar x = view.getFieldAs<Scalar>(Dimension::Id::X, id);
                    Scalar y = view.getFieldAs<Scalar>(Dimension::Id::Y, id);
                    Scalar z = view.getFieldAs<Scalar>(Dimension::Id::Z, id);

                    view.setField(pdal::Dimension::Id::X, id, transform (0, 0) * x + transform (0, 1) * y + transform (0, 2) * z + transform (0, 3));
                    view.setField(pdal::Dimension::Id::Y, id, transform (1, 0) * x + transform (1, 1) * y + transform (1, 2) * z + transform (1, 3));
                    view.setField(pdal::Dimension::Id::Z, id, transform (2, 0) * x + transform (2, 1) * y + transform (2, 2) * z + transform (2, 3));
                }
            }

        // TODO: implement streaming mode
    };
}