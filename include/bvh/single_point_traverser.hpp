#ifndef BVH_SINGLE_POINT_TRAVERSAL_HPP
#define BVH_SINGLE_POINT_TRAVERSAL_HPP


#include "bvh/bvh.hpp"
#include "bvh/vector.hpp"
#include "bvh/utilities.hpp"

namespace bvh {

/// Single point-distance traversal algorithm
template <typename Bvh, size_t StackSize = 64, typename NodeIntersector = FastNodeIntersector<Bvh>>
class SinglePointTraverser {
    using Scalar = typename Bvh::ScalarType;

public:
	using Callback = std::function<bool(const FeaturePtr&, const Vector&, Result&)>;

    SinglePointTraverser(const Bvh& bvh)
        : bvh(bvh)
    {}

	/// Searches the BVH with the given point and intersector.
    template <typename PrimitiveRanger>
    bvh_always_inline
    std::optional<typename PrimitiveRanger::Result>
    traverse(const Vector3<Scalar>& pt, PrimitiveRanger& primitive_ranger, Callback& callback = []{return false;}) const {
        return query(pt, primitive_ranger, callback);
    }

private:
    static constexpr size_t stack_size = StackSize;

	struct Stack {
        using Element = typename Bvh::IndexType;

        Element elements[stack_size];
        size_t size = 0;

        void push(const Element& t) {
            assert(size < stack_size);
            elements[size++] = t;
        }

        Element pop() {
            assert(!empty());
            return elements[--size];
        }

        bool empty() const { return size == 0; }
    };

private:
    template <typename PrimitiveRanger, typename Statistics>
    bvh_always_inline
    std::optional<typename PrimitiveRanger::Result>
    query(Vector3<Scalar> pt, PrimitiveRanger& primitive_ranger, Callback& callback) const {
		auto result = std::optional<typename PrimitiveRanger::Result>(std::nullopt);

        // If the root is a leaf, intersect it and return
        if (bvh_unlikely(bvh.nodes[0].is_leaf()))
            return query_leaf(bvh.nodes[0], pt, result, primitive_ranger, callback);

        Stack stack;
        auto* left_child = &bvh.nodes[bvh.nodes[0].first_child_or_primitive];
        while (true) {
            auto* right_child = left_child + 1;
            auto distance_left  = query_node(*left_child,  pt);
            auto distance_right = query_node(*right_child, pt);

			if (distance_left < distance_right) {



				if (bvh_unlikely(left_child->is_leaf())) {
                    if (intersect_leaf(*left_child, pt, result, primitive_ranger, statistics) &&
                        primitive_ranger.any_hit)
                        break;
                    left_child = nullptr;
				}
			}

		}

	}

	/// Return signed distance-squared to surface of Node bounds
	bvh_always_inline
    Scalar query_node(const typename Bvh::Node& node, const Vector3<Scalar>& pt)
	{
		return node.bounding_box_proxy().signed_distance_sqr(pt);
	}

	template <typename PrimitiveRanger, typename Statistics>
    bvh_always_inline
    std::optional<typename PrimitiveRanger::Result>& query_leaf(
        const typename Bvh::Node& node,
        Vector3<Scalar>& pt,
        std::optional<typename PrimitiveRanger::Result>& nearest,
        PrimitiveRanger& primitive_ranger,
        Statistics& statistics) const
    {
	}



private:
    const Bvh& bvh;


};


}

#endif