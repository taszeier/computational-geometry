#pragma once

namespace compg {

    template <typename ValueType>
    struct Interval {
        Interval(ValueType lower, ValueType upper)
            : Lower(lower)
            , Upper(upper) {
            COMPG_ASSERT(lower <= upper, "Received invalid endpoints");
        }

        ValueType Lower;
        ValueType Upper;
    };

    template <typename ValueType>
    constexpr auto make_interval(ValueType lower, ValueType upper) {
        return Interval<ValueType>{lower, upper};
    }

    template <typename ValueType>
    constexpr auto interval(ValueType lower, ValueType upper) {
        return make_interval(lower, upper);
    }

    struct OpenInterval {
        struct EndPoint {
            Float Value;
            bool Open;

            constexpr bool operator==(const EndPoint& other) const {
                return Value == other.Value && Open == other.Open;
            }
        };

        struct LowerEndPointLess {
            static constexpr bool operator()(const EndPoint& lhs, const EndPoint& rhs) noexcept {
                return lhs.Value < rhs.Value || (lhs.Value == rhs.Value && !lhs.Open && rhs.Open);
            }

            static constexpr auto Compare(const auto& lhs, const auto& rhs) noexcept {
                return operator()(lhs, rhs);
            }
        };

        struct UpperEndPointLess {
            static constexpr bool operator()(const EndPoint& lhs, const EndPoint& rhs) noexcept {
                return lhs.Value < rhs.Value || (lhs.Value == rhs.Value && lhs.Open && !rhs.Open);
            }

            static constexpr auto Compare(const auto& lhs, const auto& rhs) noexcept {
                return operator()(lhs, rhs);
            }
        };

        /*
        struct LowerEndPoint {
            constexpr bool operator<(const LowerEndPoint& other) const {
                return EndPoint.Value < other.EndPoint.Value
                    || (EndPoint.Value == other.EndPoint.Value && !EndPoint.Open && other.EndPoint.Open);
            }

            constexpr bool operator==(const LowerEndPoint& other) const {
                return EndPoint == other.EndPoint;
            }

            constexpr bool operator<=(const LowerEndPoint& other) const {
                return *this < other || *this == other;
            }

            EndPoint EndPoint;
        };

        struct UpperEndPoint {
            constexpr bool operator<(const UpperEndPoint& other) const {
                return EndPoint.Value < other.EndPoint.Value
                    || (EndPoint.Value == other.EndPoint.Value && EndPoint.Open && !other.EndPoint.Open);
            }

            constexpr bool operator==(const UpperEndPoint& other) const {
                return EndPoint == other.EndPoint;
            }

            constexpr bool operator<=(const UpperEndPoint& other) const {
                return *this < other || *this == other;
            }

            EndPoint EndPoint;
        };

        */

        constexpr OpenInterval(Float lower, Float upper, bool openLower = true, bool openUpper = true)
            : Lower{lower, openLower}
            , Upper{upper, openUpper} {
            COMPG_ASSERT(Lower.Value <= Upper.Value, "Received invalid endpoints");
        }
        constexpr OpenInterval(const EndPoint& lower, const EndPoint& upper)
            : Lower(lower)
            , Upper(upper) {
            COMPG_ASSERT(Lower.Value <= Upper.Value, "Received invalid endpoints");
        }

        EndPoint Lower;
        EndPoint Upper;

        constexpr bool Contains(Float value) const {
            const EndPoint point{value, false};
            return !LowerEndPointLess::Compare(point, Lower) && !UpperEndPointLess::Compare(Upper, point);
        }

        constexpr bool IsEmpty() const {
            return Lower.Value == Upper.Value && Lower.Open && Upper.Open;
        }

        constexpr bool IsDegenerate() const {
            return Lower.Value == Upper.Value && !Lower.Open && !Upper.Open;
        }

        static OpenInterval CreateEmpty() {
            return OpenInterval{0, 0};
        }
    };

    constexpr bool UnionIsInterval(const OpenInterval& interval1, const OpenInterval& interval2) {
        const auto lower = std::max(interval1.Lower, interval2.Lower, OpenInterval::LowerEndPointLess{});
        const auto upper = std::min(interval1.Upper, interval2.Upper, OpenInterval::UpperEndPointLess{});
        return lower.Value < upper.Value || (lower.Value == upper.Value && !(lower.Open && upper.Open));
    }

    constexpr OpenInterval Union(const OpenInterval& interval1, const OpenInterval& interval2) {
        COMPG_ASSERT(UnionIsInterval(interval1, interval2), "Received invalid intervals");
        const auto lower = std::min(interval1.Lower, interval2.Lower, OpenInterval::LowerEndPointLess{});
        const auto upper = std::max(interval1.Upper, interval2.Upper, OpenInterval::UpperEndPointLess{});

        return OpenInterval{lower, upper};
    }

    constexpr bool Covers(const OpenInterval& interval1, const OpenInterval& interval2) {
        if (interval2.IsEmpty()) {
            return true;
        }
        if (interval1.IsEmpty()) {
            return false;
        }
        return !OpenInterval::LowerEndPointLess::Compare(interval2.Lower, interval1.Lower)
               && !OpenInterval::UpperEndPointLess::Compare(interval1.Upper, interval2.Upper);
    }

    constexpr OpenInterval Intersection(const OpenInterval& interval1, const OpenInterval& interval2) {
        const auto lower = std::max(interval1.Lower, interval2.Lower, OpenInterval::LowerEndPointLess{});
        const auto upper = std::min(interval1.Upper, interval2.Upper, OpenInterval::UpperEndPointLess{});
        return (lower.Value <= upper.Value) ? OpenInterval{lower, upper} : OpenInterval::CreateEmpty();
    }

    constexpr bool Intersects(const OpenInterval& interval1, const OpenInterval& interval2) {
        return !Intersection(interval1, interval2).IsEmpty();
    }

} // namespace compg