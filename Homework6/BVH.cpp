#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}
float getDimValue(int dim, Vector3f v) {
    if (dim == 0) return v.x;
    else if (dim == 1) return v.y;
    else return v.z;
}
BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }
        double a,b;
        double pdim_min,pdim_max;
        switch (dim) {
        case 0:
            pdim_min=centroidBounds.pMin.x;
            pdim_max=centroidBounds.pMax.x;
            a=centroidBounds.pMax.y-centroidBounds.pMin.y;
            b=centroidBounds.pMax.z-centroidBounds.pMin.z;
            break;
        case 1:
            pdim_min=centroidBounds.pMin.y;
            pdim_max=centroidBounds.pMax.y;
            a=centroidBounds.pMax.x-centroidBounds.pMin.x;
            b=centroidBounds.pMax.z-centroidBounds.pMin.z;
            break;
        case 2:
            pdim_min=centroidBounds.pMin.z;
            pdim_max=centroidBounds.pMax.z;
            a=centroidBounds.pMax.x-centroidBounds.pMin.x;
            b=centroidBounds.pMax.y-centroidBounds.pMin.y;
            break;
        }
        int bucknum=6;
        double minC=std::numeric_limits<float>::infinity();
        int pviot=0;
        double bucksize=(pdim_max-pdim_min)/bucknum;
        for (int i=1;i<bucknum;i++){
            int j=0;
            for (j=0; j < objects.size(); j++)
            {
                if(getDimValue(dim,objects[j]->getBounds().Centroid())<pdim_min+bucksize*i){
                    continue;
                }
                else{
                    break;
                }
            }
            double tmpC=j*(a*b+a*bucksize*i+b*bucksize*i)+(objects.size()-j)*(a*b+(bucknum-i)*a*bucksize+(bucknum-i)*bucksize*b);
            if(tmpC<minC){
                minC=tmpC;
                pviot=j;
            }
            
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + pviot+1;
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }
    
    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    Intersection isect;
    std::array<int, 3> dirIsNeg;
    dirIsNeg[0]=int(ray.direction.x>0);
    dirIsNeg[1]=int(ray.direction.y>0);
    dirIsNeg[2]=int(ray.direction.z>0);
    if((!node->right)&&(!node->left)){   
        isect=node->object->getIntersection(ray);
        return isect;
    }
    else{
        Intersection risect;
        Intersection lisect;
        if (node->right&&node->right->bounds.IntersectP(ray,ray.direction_inv,dirIsNeg))
            risect=BVHAccel::getIntersection(node->right, ray);
        if (node->left&&node->left->bounds.IntersectP(ray,ray.direction_inv,dirIsNeg))
            lisect=BVHAccel::getIntersection(node->left, ray);
        if(!risect.happened)
            return lisect;
        else if (!lisect.happened)
            return risect;
        else 
            return risect.distance>lisect.distance?lisect:risect;
    }
    // invDir: ray direction(x,y,z), invDir=(1.0/x,1.0/y,1.0/z), use this because Multiply is faster that Division
    // dirIsNeg: ray direction(x,y,z), dirIsNeg=[int(x>0),int(y>0),int(z>0)], use this to simplify your logic
    // TODO test if ray bound intersects
    
    
    //if (node->right&&node->right->bounds.IntersectP())
    //{
    //    risect=BVHAccel::getIntersection(node->right, ray);
    //}
    //if (node->left&&node->left->bounds.IntersectP())
    //{
    //    lisect=BVHAccel::getIntersection(node->left, ray);
    //}
    //if ((!node->right)&&(!node->left))
    //{
    //    isect=BVHAccel::getIntersection(node,ray);
    //    return isect;
    //}
//
    //if ((!risect.happened)&&lisect.happened)
    //{
    //    return lisect;
    //}
    //else if((!lisect.happened)&&risect.happened){
    //    return risect;
    //}
    //else{
    //    return isect;
    //}
    
    
    
    
    

}