//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    auto p = this->intersect(ray);
    Vector3f L_dir={ 0.0,0.0,0.0 };
    Vector3f L_indir={ 0.0,0.0,0.0 };
    if(!p.happened){
        return L_dir;
    }
    else if(p.obj->hasEmit()&&depth==0)
    {
        return p.m->getEmission();
    }
    Intersection inter;
    float pdf_light=0.0f;
    sampleLight(inter,pdf_light);


    auto N      = p.normal.normalized();//
    auto wo     = ray.direction;//

    auto x      = inter.coords;//
    auto NN     = inter.normal.normalized();//
    Vector3f ws = (p.coords-x).normalized();//
    auto emit   = inter.emit;//
    float dis2 = dotProduct((p.coords - x), (p.coords - x));
    
    Ray ws_ray (p.coords,-ws);//物体
    auto block=this->intersect(ws_ray);

    if(block.hobj==inter.obj){
        Vector3f L_i = inter.emit;//光强
        Vector3f f_r = p.m->eval(wo, -ws, N);//材质，课上说了，BRDF==材质，ws不参与计算
        float cos_theta = dotProduct(-ws, N);//物体夹角
        float cos_theta_l = dotProduct(ws, NN);//光源夹角
        L_dir = L_i * f_r * cos_theta * cos_theta_l / dis2 / pdf_light;        
    }
    float f=get_random_float();
    if(f<RussianRoulette){
        auto wi=p.m->sample(wo,N).normalized();

        Ray r (p.coords,wi);//物体
        auto block_rr=this->intersect(r);
        
        if(block_rr.happened&&!block_rr.m->hasEmission()){
            L_indir=castRay(r,depth++)*p.m->eval(wo,wi,N)*dotProduct(wi,N)/p.m->pdf(wo,wi,N)/RussianRoulette;
        }
    }
    return L_dir+L_indir;
    
}