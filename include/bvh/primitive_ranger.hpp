#ifndef BVH_PRIMITIVE_RANGER_HPP
#define BVH_PRIMITIVE_RANGER_HPP

#include <optional>

namespace bvh {

/// Base class for primitive rangers (computes distance (i.e. "range") to nearest primitive)
template <typename Bvh, typename Primitive, bool Permuted>
struct PrimitiveRanger {
    PrimitiveRanger(const Bvh& bvh, const Primitive* primitives)
        : bvh(bvh), primitives(primitives)
    {}

    const Bvh& bvh;
    const Primitive* primitives = nullptr;
};

/// A ranger that looks for the closest primitive.
template <typename Bvh, typename Primitive, bool Permuted = false>
struct ClosestPrimitiveRanger : public PrimitiveRanger<Bvh, Primitive, Permuted> {
    using Scalar       = typename Primitive::ScalarType;
    using Range        = typename Primitive::RangeType;

    struct Result {
        size_t       primitive_index;
        Range        range;

        Scalar distance() const { return range.distance(); }
    };

    ClosestPrimitiveRanger(const Bvh& bvh, const Primitive* primitives)
        : PrimitiveRanger<Bvh, Primitive, Permuted, false>(bvh, primitives)
    {}

    std::optional<Result> range(size_t index, const Vector3<Scalar>& pt) const {
        auto [p, i] = this->primitive_at(index);
        if (auto hit = p.range(pt))
            return std::make_optional(Result { i, *hit });
        return std::nullopt;
    }
};

}

#endif