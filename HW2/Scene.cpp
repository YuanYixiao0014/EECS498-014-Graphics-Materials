#include "Scene.h"
#include "Config.h"
#include <iostream>
#include <filesystem>

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

Vec3 Scene::trace(const Ray &ray, int bouncesLeft, bool discardEmission) {
    if constexpr(DEBUG) {
        assert (ray.isNormalized());
    }
    if (bouncesLeft < 0) return {};
    // TODO...
    //get intersection
    Intersection intersec = getIntersection(ray);
    if(!intersec.happened) return {};       //no intersection, return
    Vec3 irradiance = Vec3(0.0f, 0.0f, 0.0f);
    Vec3 emission = intersec.getEmission();   //surface emission

    //for direct radiance
    Intersection sample_light = sampleLight();
    Vec3 lightDir = sample_light.pos - intersec.pos;
    float lightDist = lightDir.getLength();
    lightDir.normalize();
    Ray lightRay = {intersec.pos, lightDir};
    Intersection intersec1 = getIntersection(lightRay);
    Vec3 directRadiance = Vec3(0.0f, 0.0f, 0.0f);
    if(intersec1.happened){
        //calculate Direct Illumination
        float pdf = 1/(lightArea);
        float cosTheta = lightRay.dir.dot(intersec.getNormal());
        float cosTheta1 = (-sample_light.getNormal()).dot(lightRay.dir);
        Vec3 brdf_light = intersec.calcBRDF(-lightRay.dir, -ray.dir);
        directRadiance = (1/pdf) * intersec1.getEmission() * brdf_light 
                * cosTheta * cosTheta1 / (lightDist * lightDist);
    }
    //for indirect radiance
    Vec3 direction = Random::cosWeightedHemisphere(intersec.getNormal());
    Ray nextRay = {intersec.pos, direction};
    Intersection intersec2 = getIntersection(nextRay);
    if(intersec2.happened){
        //calculate indirect Illumination
        float pdf = (intersec.getNormal().dot(direction))/(PI);
        float cosTheta = nextRay.dir.dot(intersec.getNormal());
        Vec3 brdf = intersec.calcBRDF(-nextRay.dir, -ray.dir);
        //trace the ray without the emission:
        Vec3 indirectRadiance = (1/pdf) 
            * trace(nextRay, bouncesLeft - 1, true) * brdf * cosTheta;
        if(discardEmission){
            irradiance = directRadiance + indirectRadiance ;
        }else{
            irradiance = emission + directRadiance + indirectRadiance;
        }
    }

    return irradiance;
}

tinyobj::ObjReader Scene::reader {};

void Scene::addObjects(std::string_view modelPath, std::string_view searchPath) {
    tinyobj::ObjReaderConfig config;
    config.mtl_search_path = searchPath;
    if (!reader.ParseFromFile(std::string(modelPath), config)) {
        if (!reader.Error().empty()) {
            std::cerr << "TinyObjReader: " << reader.Error();
            std::filesystem::path relative(modelPath);
            std::cerr << "Reading file " << std::filesystem::absolute(relative) << " error. File may be malformed or not exist.\n";
        }
        exit(1);
    }

    auto& attrib = reader.GetAttrib();
    auto& shapes = reader.GetShapes();
    auto& materials = reader.GetMaterials();

    // Loop over shapes
    for (size_t s = 0; s < shapes.size(); s++) {
        auto* object = new Object();
        object->name = shapes[s].name;
        // Loop over faces(polygon)
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            size_t fv = size_t(shapes[s].mesh.num_face_vertices[f]);
            std::vector<Vec3> positions;
            // Loop over vertices in the face.
            for (size_t v = 0; v < fv; v++) {
                // access to vertex
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                tinyobj::real_t vx = attrib.vertices[3*size_t(idx.vertex_index)+0];
                tinyobj::real_t vy = attrib.vertices[3*size_t(idx.vertex_index)+1];
                tinyobj::real_t vz = attrib.vertices[3*size_t(idx.vertex_index)+2];

                positions.push_back({vx, vy, vz});
            } // per-vertex
            index_offset += fv;
            Mesh mesh {positions[0], positions[1], positions[2]};
            object->area += mesh.area;
            object->meshes.push_back(std::move(mesh));
        } // per-face
        object->constructBoundingBox();
        // we assume each object uses only a single material for all meshes
        auto materialId = shapes[s].mesh.material_ids[0];
        auto& material = materials[materialId];
        object->kd = Vec3 {
            material.diffuse[0],
            material.diffuse[1],
            material.diffuse[2],
        };
        if (material.emission[0] + 
            material.emission[1] + 
            material.emission[2] > 0) { // is light
            object->ke = Vec3 {
                material.emission[0], 
                material.emission[1],
                material.emission[2]
            };
            object->hasEmission = true;
            lights.push_back(object);
            lightArea += object->area;
        }
        objects.push_back(object);
    } // per-shape
}

void Scene::constructBVH() {
    assert (!objects.empty());
    bvh.root = BVH::build(objects);
}

Intersection Scene::getIntersection(const Ray &ray) {
    assert (bvh.root);
    return bvh.root->intersect(ray);
}

Intersection Scene::sampleLight() const {
    assert (lights.size() == 1 && "Currently only support a single light object");
    assert (lightArea > 0.0f);
    Intersection inter;
    return lights[0]->sample();
}

Scene::~Scene() {
    for (auto obj : objects) {
        delete obj;
    }
}
