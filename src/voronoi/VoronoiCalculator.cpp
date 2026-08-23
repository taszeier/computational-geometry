#include "voronoi/VoronoiCalculator.hpp"

#include "data_structures/Cropper.hpp"
#include "data_structures/DoublyConnectedEdgeListAlgorithms.hpp"
#include "data_structures/PriorityQueue.hpp"
#include "math/Geometry.hpp"
#include "sweep_line/SweepLine.hpp"
#include "voronoi/BeachLine.hpp"

namespace compg {
    namespace details {
        using event_id_type = std::variant<SiteEventId, CircleEventId>;

        auto Initialize(const auto& sites, auto& eventQueue, auto& halfLineOrigin) {
            auto getY = [&sites](event_id_type e) { return sites.at(std::get<0>(e).Id)[1]; };

            std::vector<SiteEventId> collinearSites{std::get<0>(eventQueue.Top())};
            eventQueue.Pop();
            while (!eventQueue.IsEmpty()
                   && math::IsZero(getY(eventQueue.Top()) - sites.at(collinearSites.at(0).Id)[1])) {
                collinearSites.push_back(std::get<0>(eventQueue.Top()));
                eventQueue.Pop();
            }

            BeachLine beachLine{sites, collinearSites};
            if (eventQueue.IsEmpty()) {
                std::ranges::for_each(
                    collinearSites | std::views::slide(2), [&sites, &halfLineOrigin](const auto& iterable) {
                        const auto a = *iterable.begin();
                        const auto b = *std::next(iterable.begin());
                        const auto middle = (sites.at(a.Id) + sites.at(b.Id)) * 0.5;
                        halfLineOrigin[BreakPoint{a, b}] = middle;
                        halfLineOrigin[BreakPoint{b, a}] = middle;
                    }
                );
            } else {
                const Float sweepLineY = getY(eventQueue.Top());
                std::ranges::for_each(
                    collinearSites | std::views::slide(2), [&sites, &halfLineOrigin, sweepLineY](const auto& iterable) {
                        const auto a = *iterable.begin();
                        const auto b = *std::next(iterable.begin());
                        const Float x = (sites.at(a.Id)[0] + sites.at(b.Id)[0]) * 0.5;
                        const auto poly = CreateSitePolynomial(sites.at(a.Id), sweepLineY);
                        const Float y = poly.Evaluate(x);
                        halfLineOrigin[BreakPoint{a, b}] = Vertex2D{x, y};
                        halfLineOrigin[BreakPoint{b, a}] = Vertex2D{x, y};
                    }
                );
            }

            return beachLine;
        }

        DoublyConnectedEdgeList
        Crop(const auto& sites, const auto& halfEdgeOrigin, const auto& segments, const Box2D& box) {
            const auto halfLines = halfEdgeOrigin | std::views::transform([&sites](const auto& tup) {
                                       const auto& [breakPoint, origin] = tup;
                                       const auto d
                                           = sites.at(breakPoint.RightSite.Id) - sites.at(breakPoint.LeftSite.Id);
                                       const Vertex2D n{d[1], -d[0]};
                                       return HalfLine2D{origin, n};
                                   })
                                   | std::ranges::to<std::vector>();
            const Cropper2D cropper{box};
            return cropper.Crop(segments, halfLines);
        }

        class VoronoiCalculatorState {
        public:
            using sites_type = std::vector<Vertex2D>;
            using circle_event_break_points_type = std::unordered_map<
                CircleEventId,
                std::tuple<BeachLine::break_points_type::iterator, BeachLine::break_points_type::iterator>>;
            using circle_event_location_type = std::unordered_map<CircleEventId, Vertex2D>;
            using circle_event_center_type = std::unordered_map<CircleEventId, Vertex2D>;

            explicit VoronoiCalculatorState(const sites_type& sites)
                : Sites(sites)
                , EventQueue{std::views::iota(0UZ, sites.size()) | std::views::transform([](const auto i) { return SiteEventId{i}; }), EventBelowComparator{Sites, CircleEventLocation}}
                , BeachLine{details::Initialize(Sites, EventQueue, HalfLineOrigin)}{
            }

            void RemoveCircleEvent(const CircleEventId& e) {
                CircleEventLocation.erase(e);
                CircleEventCenter.erase(e);
                CircleEventBreakPoints.erase(e);
                EventQueue.Erase(e);
            }

            void AddCircleEvent(auto leftIt, auto rightIt) {
                COMPG_ASSERT(
                    leftIt->RightSite == rightIt->LeftSite, "The iterators do not define a potential circle event"
                );

                auto toBisector = [this](const BreakPoint& breakPoint) {
                    const auto origin = HalfLineOrigin.at(breakPoint);
                    const auto v0 = Sites.at(breakPoint.LeftSite.Id);
                    const auto v1 = Sites.at(breakPoint.RightSite.Id);
                    const auto diff = v1 - v0;
                    const Vertex2D direction{diff[1], -diff[0]};
                    return HalfLine2D{origin, direction};
                };
                if (const auto circumcenter = FindIntersection(toBisector(*leftIt), toBisector(*rightIt));
                    circumcenter) {
                    const auto v0 = Sites.at(leftIt->LeftSite.Id);
                    const Float radius = (circumcenter.value() - v0).norm();

                    const CircleEventId event{{leftIt->LeftSite, leftIt->RightSite, rightIt->RightSite}};
                    CircleEventCenter[event] = circumcenter.value();
                    CircleEventLocation[event] = Vertex2D{circumcenter.value()[0], circumcenter.value()[1] - radius};
                    CircleEventBreakPoints.insert(std::pair{event, std::tuple{leftIt, rightIt}});
                    EventQueue.Insert(event);
                }
            }

            void HandleSiteEvent(SiteEventId event) {
                const auto arc = BeachLine.FindArcAbove(Sites.at(event.Id));

                const auto poly = CreateSitePolynomial(Sites.at(arc.MiddleSite.Id), Sites.at(event.Id)[1]);
                const auto x = Sites.at(event.Id)[0];
                const auto y = poly.Evaluate(x);
                HalfLineOrigin[BreakPoint{.LeftSite = arc.MiddleSite, .RightSite = event}] = Vertex2D{x, y};
                HalfLineOrigin[BreakPoint{.LeftSite = event, .RightSite = arc.MiddleSite}] = Vertex2D{x, y};

                RemoveCircleEvent({arc});
                const auto [leftIt, rightIt] = BeachLine.InsertArc(event);
                if (arc.LeftSite.has_value()) {
                    AddCircleEvent(std::prev(leftIt), leftIt);
                }
                if (arc.RightSite.has_value()) {
                    AddCircleEvent(rightIt, std::next(rightIt));
                }
            }

            void HandleCircleEvent(const CircleEventId& event) {
                const auto& arc = event.Arc;
                COMPG_ASSERT(arc.LeftSite.has_value() && arc.RightSite.has_value(), "Expected a valid circle event");

                const auto v0 = CircleEventCenter.at(event);

                const BreakPoint leftHalfEdge{.LeftSite = arc.LeftSite.value(), .RightSite = arc.MiddleSite};
                const auto leftOrigin = HalfLineOrigin.at(leftHalfEdge);
                if (leftOrigin != v0) {
                    Segments.emplace_back(leftOrigin, v0);
                }
                HalfLineOrigin.erase(leftHalfEdge);

                const BreakPoint rightHalfEdge{.LeftSite = arc.MiddleSite, .RightSite = arc.RightSite.value()};
                const auto rightOrigin = HalfLineOrigin.at(rightHalfEdge);
                if (rightOrigin != v0) {
                    Segments.emplace_back(rightOrigin, v0);
                }
                HalfLineOrigin.erase(rightHalfEdge);
                HalfLineOrigin[BreakPoint{.LeftSite = arc.LeftSite.value(), .RightSite = arc.RightSite.value()}] = v0;

                const auto [left, right] = CircleEventBreakPoints.at(event);
                const auto [farLeft, middle, farRight] = BeachLine.EraseArc(left, right);
                RemoveCircleEvent(event);
                if (farLeft.has_value()) {
                    const Arc oldLeftArc{
                        .LeftSite = farLeft.value()->LeftSite,
                        .MiddleSite = arc.LeftSite.value(),
                        .RightSite = arc.MiddleSite
                    };
                    RemoveCircleEvent({oldLeftArc});
                    AddCircleEvent(farLeft.value(), middle);
                }
                if (farRight.has_value()) {
                    const Arc oldRightArc{
                        .LeftSite = arc.MiddleSite,
                        .MiddleSite = arc.RightSite.value(),
                        .RightSite = farRight.value()->RightSite
                    };
                    RemoveCircleEvent({oldRightArc});
                    AddCircleEvent(middle, farRight.value());
                }
            }

            void Run() {
                while (!EventQueue.IsEmpty()) {
                    const auto eventId = EventQueue.Top();
                    EventQueue.Pop();
                    std::visit(
                        Overloads{
                            [this](SiteEventId e) { HandleSiteEvent(e); },
                            [this](const CircleEventId& e) { HandleCircleEvent(e); }
                        },
                        eventId
                    );
                }
            }

            DoublyConnectedEdgeList GetResult(const Box2D& box) const {
                return details::Crop(Sites, HalfLineOrigin, Segments, box);
            }

        private:
            struct EventBelowComparator {
                bool operator()(event_id_type lhs, event_id_type rhs) const {
                    const auto lhsLoc = GetLocation(lhs);
                    const auto rhsLoc = GetLocation(rhs);
                    return VertexBelowComparator::Compare(lhsLoc, rhsLoc);
                };

                [[nodiscard]] Vertex2D GetLocation(event_id_type id) const {
                    return std::visit(
                        Overloads{
                            [this](SiteEventId e) { return Sites.at(e.Id); },
                            [this](CircleEventId e) { return CircleEventLocation.at(e); }
                        },
                        id
                    );
                }

                const sites_type& Sites;
                const circle_event_location_type& CircleEventLocation;
            };

            const sites_type& Sites;
            circle_event_break_points_type CircleEventBreakPoints;
            circle_event_location_type CircleEventLocation;
            circle_event_center_type CircleEventCenter;
            PriorityQueue<event_id_type, EventBelowComparator> EventQueue;
            std::unordered_map<BreakPoint, Vertex2D> HalfLineOrigin;
            std::vector<LineSegment2D> Segments;
            BeachLine BeachLine;
        };
    } // namespace details

    DoublyConnectedEdgeList
    VoronoiCalculator::FindVoronoiDiagram(const std::vector<Vertex2D>& sites, const Box2D& box) const {
        if (sites.empty()) {
            return {};
        }

        details::VoronoiCalculatorState state{sites};
        state.Run();
        return state.GetResult(box);
    }
} // namespace compg
