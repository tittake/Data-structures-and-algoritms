// Datastructures.cc

#include "datastructures.hh"
#include <random>
#include <cmath>
#include <iostream>

std::minstd_rand rand_engine; // Reasonably quick pseudo-random generator

template <typename Type>
Type random_in_range(Type start, Type end)
{
    auto range = end-start;
    ++range;

    auto num = std::uniform_int_distribution<unsigned long int>(0, range-1)(rand_engine);

    return static_cast<Type>(start+num);
}

bool comp(std::pair<Coord, PlaceID> a, std::pair<Coord, PlaceID> b){
    
    if(sqrt((pow(a.first.x,2)) + (pow(a.first.y,2))) < sqrt((pow(b.first.x,2)) + (pow(b.first.y,2)))){
        return true;
    }else if(sqrt((pow(a.first.x,2)) + (pow(a.first.y,2))) == sqrt((pow(b.first.x,2)) + (pow(b.first.y,2))))
    {
        return a.first.y <= b.first.y;
    }else{
        return false;
    }
}
// Modify the code below to implement the functionality of the class.
// Also remove comments from the parameter names when you implement
// an operation (Commenting out parameter name prevents compiler from
// warning about unused parameters on operations you haven't yet implemented.)

Datastructures::Datastructures()
{


}

Datastructures::~Datastructures()
{
    // Replace this comment with your implementation
}

int Datastructures::place_count()
{
    return placeIDs_.size();
}

void Datastructures::clear_all()
{
    places_.clear();
    placeIDs_.clear();
}

std::vector<PlaceID> Datastructures::all_places()
{
    return placeIDs_;
}

bool Datastructures::add_place(PlaceID id, const Name& name, PlaceType type, Coord xy)
{
    if(places_.find(id) == places_.end()){
        places_.insert({id, {name, xy, type}});
        placeIDs_.push_back(id);
        return true;
    }else{
        return false;
    }



}

std::pair<Name, PlaceType> Datastructures::get_place_name_type(PlaceID id)
{

    if(places_.find(id) != places_.end()){

        return {places_[id].name_, places_[id].type_};
    }else{
        return {NO_NAME, PlaceType::NO_TYPE};
    }
    }


Coord Datastructures::get_place_coord(PlaceID id)
{

    if(places_.find(id) != places_.end()){

        return {places_[id].coords_};
    }else{
        return NO_COORD;
    }
}
bool Datastructures::add_area(AreaID id, const Name &name, std::vector<Coord> coords)
{
    if(areas_.find(id) == areas_.end()){
        areas_.insert({id, {name, coords}});
        return true;
    }else{
        return false;
    }
}

Name Datastructures::get_area_name(AreaID id)
{
    if(areas_.find(id) != areas_.end()){

        return {areas_[id].name_};
    }else{
        return {NO_NAME};
    }
}

std::vector<Coord> Datastructures::get_area_coords(AreaID id)
{
    if(areas_.find(id) != areas_.end()){

        return {areas_[id].coords_};
    }else{
        return {NO_COORD};
    }
}

void Datastructures::creation_finished()
{
    // Replace this comment with your implementation
    // NOTE!! It's quite ok to leave this empty, if you don't need operations
    // that are performed after all additions have been done.
}


std::vector<PlaceID> Datastructures::places_alphabetically()
{
    std::multimap<Name, PlaceID> placeholder;
    for(auto place : places_){
        placeholder.insert({place.second.name_, place.first});
    }

    std::vector<PlaceID> ids_alphabetical;
    for(auto name : placeholder){
        ids_alphabetical.push_back(name.second);
    }
    return ids_alphabetical;
}

std::vector<PlaceID> Datastructures::places_coord_order()
{
    std::vector<std::pair<Coord, PlaceID>> placeholder;
    for(auto place : places_){
        placeholder.push_back({place.second.coords_, place.first});
    }

    std::sort(placeholder.begin(), placeholder.end(), comp);

    std::vector<PlaceID> ids_coordorder;
    for(auto i : placeholder){
        ids_coordorder.push_back(i.second);
    }
    return ids_coordorder;
}

std::vector<PlaceID> Datastructures::find_places_name(Name const& name)
{
    std::vector<PlaceID> names;
    for(auto place : places_){
        if(place.second.name_ == name){
            names.push_back(place.first);
        }
    }
    return names;
}

std::vector<PlaceID> Datastructures::find_places_type(PlaceType type)
{
    std::vector<PlaceID> types;
    for(auto place : places_){
        if(place.second.type_ == type){
            types.push_back(place.first);
        }
    }
    return types;
}

bool Datastructures::change_place_name(PlaceID id, const Name& newname)
{
    if(places_.find(id) != places_.end()){
        places_[id].name_ = newname;
        return true;

    }else{
        return false;
    }

}

bool Datastructures::change_place_coord(PlaceID id, Coord newcoord)
{
    if(places_.find(id) != places_.end()){
        places_[id].coords_ = newcoord;
        return true;

    }else{
        return false;
    }
}

std::vector<AreaID> Datastructures::all_areas()
{
    std::vector<AreaID> all_areas;
    for(auto area : areas_){
        all_areas.push_back(area.first);
    }
    return all_areas;
}

bool Datastructures::add_subarea_to_area(AreaID id, AreaID parentid)
{
    if(areas_.find(parentid) != areas_.end() and areas_.find(id) != areas_.end()){
        std::shared_ptr<AreaID> parent = std::make_shared<AreaID>(parentid);
        areas_[id].parent_ = parent;
        return true;
    }else if(areas_.find(id)->second.parent_ != nullptr){
        return false;
    }else{
        return false;
    }

}

std::vector<AreaID> Datastructures::subarea_in_areas(AreaID id)
{
    std::vector<AreaID> parents;

    if(areas_.find(id) != areas_.end()){
        std::shared_ptr<AreaID> parent = areas_[id].parent_;
        while(parent != nullptr){
            parents.push_back(*parent);
            parent = areas_[*parent].parent_;

        }
        return parents;
    }else{
        return {NO_AREA};
    }
}

std::vector<PlaceID> Datastructures::places_closest_to(Coord xy, PlaceType type)
{
    // Replace this comment with your implementation
    return {};
}

bool Datastructures::remove_place(PlaceID id)
{
    // Replace this comment with your implementation
    return false;
}

std::vector<AreaID> Datastructures::all_subareas_in_area(AreaID id)
{
    // Replace this comment with your implementation
    return {NO_AREA};
}

AreaID Datastructures::common_area_of_subareas(AreaID id1, AreaID id2)
{
    // Replace this comment with your implementation
    return NO_AREA;
}
