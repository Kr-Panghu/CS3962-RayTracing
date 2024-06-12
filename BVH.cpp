#include <algorithm>
#include <cassert>
#include "BVH.hpp"
#include "global.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int employ_SAH, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives, employ_SAH);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects, int employ_SAH)
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
        node->left = recursiveBuild(std::vector{objects[0]}, employ_SAH);
        node->right = recursiveBuild(std::vector{objects[1]}, employ_SAH);

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

        auto beginning = objects.begin();
        auto ending = objects.end();
        auto middling = objects.begin();
        std::vector<Object*> leftshapes;
        std::vector<Object*> rightshapes;   
        
        if (employ_SAH == 0) {
            middling = objects.begin() + (objects.size() / 2);
            leftshapes = std::vector<Object*>(beginning, middling);
            rightshapes = std::vector<Object*>(middling, ending);
        } else {
            float min_cost = std::numeric_limits<float>::infinity(); // Initialize with infinity to ensure first cost update
            Bounds3 a;
            Bounds3 b;

            for (int i = 0; i < objects.size() - 1; ++i) {
                for (int j = 0; j < i; ++j) {
                    a = Union(a, objects[j]->getBounds());
                }
                for (int j = i + 1; j < objects.size(); ++j) {
                    b = Union(b, objects[j]->getBounds());
                }

                float temp_cost = (i + 1) * a.SurfaceArea()+ (objects.size() - i - 1) * b.SurfaceArea();

                if (temp_cost < min_cost) {
                    middling = objects.begin() + i;
                    min_cost = temp_cost;
                }
            }
            leftshapes = std::vector<Object*>(beginning, middling);
            rightshapes = std::vector<Object*>(middling, ending);
        }

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes, employ_SAH);
        node->right = recursiveBuild(rightshapes, employ_SAH);

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
    std::array<int, 3> dirIsNeg = {int(ray.direction.x > 0), int(ray.direction.y > 0), int(ray.direction.z > 0)};
    Vector3f invDir = Vector3f(1.0f / ray.direction.x, 1.0f / ray.direction.y, 1.0f / ray.direction.z);

    // Intersects current node
    if (!node->bounds.IntersectP(ray, invDir, dirIsNeg))
        return Intersection();

    // If leaf node => return the intersection with the object
    if (node->left == nullptr && node->right == nullptr)
        return node->object->getIntersection(ray);

    // Otherwise => recurse in the child nodes
    Intersection hitLeft = getIntersection(node->left, ray);
    Intersection hitRight = getIntersection(node->right, ray);

    // Return the closer one
    if (hitLeft.distance < hitRight.distance)
        return hitLeft;
    else
        return hitRight;
}
