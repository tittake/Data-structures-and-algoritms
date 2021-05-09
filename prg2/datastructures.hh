// Datastructures.hh

#ifndef DATASTRUCTURES_HH
#define DATASTRUCTURES_HH

#include <string>
#include <vector>
#include <tuple>
#include <utility>
#include <limits>
#include <functional>
#include <set>
#include <map>
#include <math.h>
#include <algorithm>
#include <memory>
#include <stack>

// Types for IDs
using PlaceID = long long int;
using AreaID = long long int;
using Name = std::string;
using WayID = std::string;

// Return values for cases where required thing was not found
PlaceID const NO_PLACE = -1;
AreaID const NO_AREA = -1;
WayID const NO_WAY = "!!No way!!";

// Return value for cases where integer values were not found
int const NO_VALUE = std::numeric_limits<int>::min();

// Return value for cases where name values were not found
Name const NO_NAME = "!!NO_NAME!!";

// Enumeration for different place types
// !!Note since this is a C++11 "scoped enumeration", you'll have to refer to
// individual values as PlaceType::SHELTER etc.
enum class PlaceType { OTHER=0, FIREPIT, SHELTER, PARKING, PEAK, BAY, AREA, NO_TYPE };

// Type for a coordinate (x, y)
struct Coord
{
    int x = NO_VALUE;
    int y = NO_VALUE;
};

// Example: Defining == and hash function for Coord so that it can be used
// as key for std::unordered_map/set, if needed
inline bool operator==(Coord c1, Coord c2) { return c1.x == c2.x && c1.y == c2.y; }
inline bool operator!=(Coord c1, Coord c2) { return !(c1==c2); } // Not strictly necessary

struct CoordHash
{
    std::size_t operator()(Coord xy) const
    {
        auto hasher = std::hash<int>();
        auto xhash = hasher(xy.x);
        auto yhash = hasher(xy.y);
        // Combine hash values (magic!)
        return xhash ^ (yhash + 0x9e3779b9 + (xhash << 6) + (xhash >> 2));
    }
};

// Example: Defining < for Coord so that it can be used
// as key for std::map/set
inline bool operator<(Coord c1, Coord c2)
{
    if (c1.y < c2.y) { return true; }
    else if (c2.y < c1.y) { return false; }
    else { return c1.x < c2.x; }
}

// Return value for cases where coordinates were not found
Coord const NO_COORD = {NO_VALUE, NO_VALUE};

// Type for a distance (in metres)
using Distance = long long int;

// Return value for cases where Duration is unknown
Distance const NO_DISTANCE = NO_VALUE;



// This is the class you are supposed to implement

class Datastructures
{
public:
    Datastructures();
    ~Datastructures();

    // Estimate of performance: O(1)
    // Short rationale for estimate: Method only returns the size of a vector.
    int place_count();

    // Estimate of performance: θ(n)
    // Short rationale for estimate: clear is a linear method
    void clear_all();

    // Estimate of performance: O(1)
    // Short rationale for estimate: Method only returns a vector.
    std::vector<PlaceID> all_places();

    // Estimate of performance: O(n) (≈θ(1))
    // Short rationale for estimate: Find-algorithm is O(n)≈θ(1), insert is ≈θ(1)
    // and push_back is O(1).
    bool add_place(PlaceID id, Name const& name, PlaceType type, Coord xy);

    // Estimate of performance: O(n) (≈θ(1))
    // Short rationale for estimate: At its worst find-algorithm's efficiency is O(n)
    // for unordered map, but most times its a constant
    std::pair<Name, PlaceType> get_place_name_type(PlaceID id);

    // Estimate of performance: O(n) (≈θ(1))
    // Short rationale for estimate: At its worst find-algorithm's efficiency is O(n)
    // for unordered map, but most times its a constant
    Coord get_place_coord(PlaceID id);

    // We recommend you implement the operations below only after implementing the ones above

    // Estimate of performance: θ(n)
    // Short rationale for estimate: For-loops make the process linear.
    std::vector<PlaceID> places_alphabetically();

    // Estimate of performance: O(n log(n))
    // Short rationale for estimate: Sort-algorithm is O(n log(n)).
    std::vector<PlaceID> places_coord_order();

    // Estimate of performance: θ(n)
    // Short rationale for estimate: For-loops make the process linear.
    std::vector<PlaceID> find_places_name(Name const& name);

    // Estimate of performance: θ(n)
    // Short rationale for estimate: For-loops make the process linear.
    std::vector<PlaceID> find_places_type(PlaceType type);

    // Estimate of performance: O(n) (≈θ(1))
    // Short rationale for estimate: At its worst find-algorithm's efficiency is O(n)
    // for unordered map, but most times its a constant
    bool change_place_name(PlaceID id, Name const& newname);

    // Estimate of performance: O(n) (≈θ(1))
    // Short rationale for estimate: At its worst find-algorithm's efficiency is O(n)
    // for unordered map, but most times its a constant
    bool change_place_coord(PlaceID id, Coord newcoord);

    // We recommend you implement the operations below only after implementing the ones above

    // Estimate of performance: O(n) (≈θ(1))
    // Short rationale for estimate: Inserting to umap is O(n) (≈θ(1)).
    bool add_area(AreaID id, Name const& name, std::vector<Coord> coords);

    // Estimate of performance: O(n) (≈θ(1))
    // Short rationale for estimate: At its worst find-algorithm's efficiency is O(n)
    // for unordered map, but most times its a constant
    Name get_area_name(AreaID id);

    // Estimate of performance: O(n) (≈θ(1))
    // Short rationale for estimate: At its worst find-algorithm's efficiency is O(n)
    // for unordered map, but most times its a constant
    std::vector<Coord> get_area_coords(AreaID id);

    // Estimate of performance: θ(n)
    // Short rationale for estimate: For-loop is linear.
    std::vector<AreaID> all_areas();

    // Estimate of performance: O(n) (≈θ(1))
    // Short rationale for estimate: At its worst find-algorithm's efficiency is O(n)
    // for unordered map, but most times its a constant
    bool add_subarea_to_area(AreaID id, AreaID parentid);

    // Estimate of performance: O(n)
    // Short rationale for estimate: Find-algorithm is ≈θ(1), while-loop is O(n)
    // and push_back is O(1). At its absolute worst this could be O(n^2)... but let's
    // not think about that.
    std::vector<AreaID> subarea_in_areas(AreaID id);

    // Non-compulsory operations

    // Estimate of performance:
    // Short rationale for estimate:
    void creation_finished();

    // Estimate of performance: O(n^2)
    // Short rationale for estimate: Insert inside a for-loop (two linear methods).
    // Most times, this is not even near to n^2, though.
    std::vector<AreaID> all_subareas_in_area(AreaID id);

    // Estimate of performance:
    // Short rationale for estimate:
    std::vector<PlaceID> places_closest_to(Coord xy, PlaceType type);

    // Estimate of performance: O(n)
    // Short rationale for estimate: Remove is a linear method and find for umap is ≈θ(1).
    bool remove_place(PlaceID id);

    // Estimate of performance: O(n^2)
    // Short rationale for estimate: At worst, the method for-loops through both parent vectors.
    AreaID common_area_of_subareas(AreaID id1, AreaID id2);

    // Phase 2 operations

    // Estimate of performance: θ(n)
    // Short rationale for estimate: For-loop through all the values.
    std::vector<WayID> all_ways();

    // Estimate of performance: O(n)
    // Short rationale for estimate: Any of the methods used aren't linear but perftest leads me to believe it's
    // linear either way.
    bool add_way(WayID id, std::vector<Coord> coords);

    // Estimate of performance: O(n)
    // Short rationale for estimate: For-loop through all the values in ways_.
    std::vector<std::pair<WayID, Coord>> ways_from(Coord xy);

    // Estimate of performance: O(n)
    // Short rationale for estimate: Find for umap is O(n)
    std::vector<Coord> get_way_coords(WayID id);

    // Estimate of performance: θ(n+m), n is elements in Ways_ and m is elements in Crossroads_
    // Short rationale for estimate: clear for umap is θ(n)
    void clear_ways();

    // Estimate of performance: O(n)
    // Short rationale for estimate: DFS is O(n)
    std::vector<std::tuple<Coord, WayID, Distance>> route_any(Coord fromxy, Coord toxy);

    // Non-compulsory operations

    // Estimate of performance:
    // Short rationale for estimate:
    bool remove_way(WayID id);

    // Estimate of performance:
    // Short rationale for estimate:
    std::vector<std::tuple<Coord, WayID, Distance>> route_least_crossroads(Coord fromxy, Coord toxy);

    // Estimate of performance:
    // Short rationale for estimate:
    std::vector<std::tuple<Coord, WayID>> route_with_cycle(Coord fromxy);

    // Estimate of performance:
    // Short rationale for estimate:
    std::vector<std::tuple<Coord, WayID, Distance>> route_shortest_distance(Coord fromxy, Coord toxy);

    // Estimate of performance:
    // Short rationale for estimate:
    Distance trim_ways();

private:

    //PRG1
    struct PlaceInfo{
        Name name_;
        Coord  coords_;
        PlaceType type_;

    };

    std::unordered_map<PlaceID, PlaceInfo> places_;
    std::vector<PlaceID> placeIDs_;

    struct AreaInfo{
        Name name_;
        std::vector<Coord> coords_;
        std::shared_ptr<AreaID> parent_ = nullptr;
        std::vector<std::shared_ptr<AreaID>> children_ = {};

    };

    std::unordered_map<AreaID, AreaInfo> areas_;

    //PRG2

    Distance count_way_length(std::vector<Coord> coords);
    struct Way{
        std::vector<Coord> coords_;
        Coord start_;
        Coord end_;
        Distance length_;

    };

    std::unordered_map<WayID, Way> ways_;

    struct Crossroad{
        Coord coord_;
        int color_;
        std::vector<std::shared_ptr<Way>> ways_connected_;
        std::shared_ptr<Crossroad> prior_;
        Distance distance_;

    };

    std::unordered_map<Coord, Crossroad, CoordHash> crossroads_;



};

#endif // DATASTRUCTURES_HH
