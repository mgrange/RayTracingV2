#include <cfloat>
#include <cmath>
#include  <ctime>
#include <algorithm>
#include <cassert>
#include  <iostream>
#include <random>

#include "vec.h"
#include "color.h"
#include "mat.h"
#include "mesh.h"
#include "wavefront.h"
#include "image.h"
#include "image_io.h"
#include "image_hdr.h"
#include "orbiter.h"

#define EPSILON 0.0001f
#define PI 3.1415926535f
#define NB_FIBO_DIR 1
#define NB_SAMPLE 1

inline Point min( const Point& a, const Point& b ) { return Point( std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z) ); }
inline Point max( const Point& a, const Point& b ) { return Point( std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z) ); }
inline float randf(){ return ( float(rand()) / float( RAND_MAX ) ); }

std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<> uniform(0.0, 1.0);

//! representation d'un rayon.
struct Ray
{
    Point o;	//!< origine.
    Vector d;	//!< direction.
    float tmax;	//!< abscisse max pour les intersections valides.

    Ray( const Point origine, const Point extremite ) : o(origine), d(Vector(origine, extremite)), tmax(1) {}
    Ray( const Point origine, const Vector direction ) : o(origine), d(direction), tmax(FLT_MAX) {}

    //!	renvoie le point a l'abscisse t sur le rayon
    Point operator( ) ( const float t ) const { return o + t * d; }
};

//! representation d'un point d'intersection.
struct Hit
{
    Point p;	    //!< position.
    Vector n;	    //!< normale.
    float t;	    //!< t, abscisse sur le rayon.
    float u, v;	    //!< u, v coordonnees barycentrique dans le triangle.
    int object_id;  //! indice du triangle dans le maillage.

    Hit( ) : p(), n(), t(FLT_MAX), u(0), v(0), object_id(-1) {}
};

struct Triangle : public TriangleData
{
    Triangle( ) : TriangleData() {}
    Triangle( const TriangleData& data ) : TriangleData(data) {}

    /* calcule l'intersection ray/triangle
        cf "fast, minimum storage ray-triangle intersection"
        http://www.graphics.cornell.edu/pubs/1997/MT97.pdf

        renvoie faux s'il n'y a pas d'intersection valide, une intersection peut exister mais peut ne pas se trouver dans l'intervalle [0 htmax] du rayon. \n
        renvoie vrai + les coordonnees barycentriques (ru, rv) du point d'intersection + sa position le long du rayon (rt). \n
        convention barycentrique : t(u, v)= (1 - u - v) * a + u * b + v * c \n
    */
    bool intersect( const Ray &ray, const float htmax, float &rt, float &ru, float&rv ) const
    {
        /* begin calculating determinant - also used to calculate U parameter */
        Vector ac= Vector(Point(a), Point(c));
        Vector pvec= cross(ray.d, ac);

        /* if determinant is near zero, ray lies in plane of triangle */
        Vector ab= Vector(Point(a), Point(b));
        float det= dot(ab, pvec);
        if(det > -EPSILON && det < EPSILON)
            return false;

        float inv_det= 1.0f / det;

        /* calculate distance from vert0 to ray origin */
        Vector tvec(Point(a), ray.o);

        /* calculate U parameter and test bounds */
        float u= dot(tvec, pvec) * inv_det;
        if(u < 0.0f || u > 1.0f)
            return false;

        /* prepare to test V parameter */
        Vector qvec= cross(tvec, ab);

        /* calculate V parameter and test bounds */
        float v= dot(ray.d, qvec) * inv_det;
        if(v < 0.0f || u + v > 1.0f)
            return false;

        /* calculate t, ray intersects triangle */
        rt= dot(ac, qvec) * inv_det;
        ru= u;
        rv= v;

        // ne renvoie vrai que si l'intersection est valide (comprise entre tmin et tmax du rayon)
        return (rt <= htmax && rt > EPSILON);
    }

    //! renvoie l'aire du triangle
    float area( ) const
    {
        return length(cross(Point(b) - Point(a), Point(c) - Point(a))) / 2.f;
    }

    //! renvoie un point a l'interieur du triangle connaissant ses coordonnees barycentriques.
    //! convention p(u, v)= (1 - u - v) * a + u * b + v * c
    Point point( const float u, const float v ) const
    {
        float w= 1.f - u - v;
        return Point(Vector(a) * w + Vector(b) * u + Vector(c) * v);
    }

    //! renvoie une normale a l'interieur du triangle connaissant ses coordonnees barycentriques.
    //! convention p(u, v)= (1 - u - v) * a + u * b + v * c
    Vector normal( const float u, const float v ) const
    {
        float w= 1.f - u - v;
        return Vector(na) * w + Vector(nb) * u + Vector(nc) * v;
    }
};

struct BBox {
  Point pmin;
  Point pmax;

  bool intersect( const Ray& ray, const float htmax, float& rtmin, float& rtmax ) const
    {
        Vector invd= Vector(1.f / ray.d.x, 1.f / ray.d.y, 1.f / ray.d.z);
        // remarque : il est un peu plus rapide de stocker invd dans la structure Ray, ou dans l'appellant / algo de parcours, au lieu de la recalculer à chaque fois

        Point rmin= pmin;
        Point rmax= pmax;
        if(ray.d.x < 0) std::swap(rmin.x, rmax.x);    // le rayon entre dans la bbox par pmax et ressort par pmin, echanger...
        if(ray.d.y < 0) std::swap(rmin.y, rmax.y);
        if(ray.d.z < 0) std::swap(rmin.z, rmax.z);

        Vector dmin= (rmin - ray.o) * invd;        // intersection avec les plans -U -V -W attachés à rmin
        Vector dmax= (rmax - ray.o) * invd;        // intersection avec les plans +U +V +W attachés à rmax
        rtmin= std::max(dmin.x, std::max(dmin.y, std::max(dmin.z, 0.f)));        // borne min de l'intervalle d'intersection
        rtmax= std::min(dmax.x, std::min(dmax.y, std::min(dmax.z, htmax)));    // borne max

        // ne renvoie vrai que si l'intersection est valide (l'intervalle n'est pas degenere)
        return (rtmin <= rtmax);
    }
};

struct Node {
  BBox bounds;
  int prime;
  int filsg;
  int filsd;
};

struct Primitive
{
    BBox bounds;
    Point center;
    int triangle_id;
};

struct predicat
{
    int axe;
    float coupe;

    predicat( const int _axe, const float _coupe ) : axe(_axe), coupe(_coupe) {}
    bool operator() ( const Primitive& p ) const
    {
        // renvoyer vrai si le centre de l'englobant de la primitive se trouve avant la coupe...
        switch(axe){
          case 0:
            return p.center.x < coupe;
          case 1:
            return p.center.y < coupe;
          case 2:
            return p.center.z < coupe;
          default:
            return false;
        }
    }
};

// ensemble de triangles
std::vector<Triangle> triangles;
std::vector<Primitive> primitives;

int build_node( std::vector<Node>& nodes,std::vector<Primitive> & prim, const int begin, const int end )
{
    if(end - begin <= 1)
    {// ensemble de triangles

        // construire une feuille qui reference la primitive d'indice begin
        // renvoyer l'indice de la feuille
        Node n;
        n.bounds = prim[begin].bounds;
        n.prime = prim[begin].triangle_id;
        n.filsd = -1;
        n.filsg = -1;
        nodes.push_back(n);

        return nodes.size() -1;

    }

    BBox bounds;
    bounds.pmin = prim[begin].center;
    bounds.pmax = prim[begin].center;
    // construire la boite englobante des centres des primitives d'indices [begin .. end[
    for(int i = begin + 1; i< end ; i ++){
      /*calcule des limites de la bbox*/
      bounds.pmin = min(bounds.pmin, prim[i].center);
      bounds.pmax = max(bounds.pmax, prim[i].center);

    }
    // trouver l'axe le plus etire de la boite englobante
    // couper en 2 au milieu de la boite englobante sur l'axe le plus etire
    //
    int axe= 0;
    float coupe = (bounds.pmax.x - bounds.pmin.x)/2;

    if(((bounds.pmax.y - bounds.pmin.y) > (bounds.pmax.x - bounds.pmin.x))
        && ((bounds.pmax.y - bounds.pmin.y) > (bounds.pmax.z - bounds.pmin.z))){
      axe = 1;
      coupe = (bounds.pmax.y - bounds.pmin.y)/2;
    }else if(((bounds.pmax.z - bounds.pmin.z) > (bounds.pmax.x - bounds.pmin.x))
            && ((bounds.pmax.z - bounds.pmin.z) > (bounds.pmax.y - bounds.pmin.y))){
      axe = 2;
      coupe = (bounds.pmax.z - bounds.pmin.z)/2;
    }


    // partitionner les primitives par rapport a la "coupe"
    Primitive *pmid= std::partition(prim.data() + begin, prim.data() + end, predicat(axe, coupe));
    int mid= std::distance(prim.data(), pmid);

    // verifier que la partition n'est pas degeneree (toutes les primitives du meme cote de la separation)
    if(mid == begin || mid == end)
      mid = (begin + end) / 2;
    assert(mid != begin);
    assert(mid != end);
    // remarque : il est possible que 2 (ou plus) primitives aient le meme centre,
    // dans ce cas, la boite englobante des centres est reduite à un point, et la partition est forcement degeneree
    // une solution est de construire une feuille,
    // ou, autre solution, forcer une repartition arbitraire des primitives entre 2 les fils, avec mid= (begin + end) / 2

    // construire le fils gauche
    int left= build_node(nodes, prim, begin, mid);

    // construire le fils droit
    int right= build_node(nodes, prim, mid, end);

    // construire un noeud interne
    // renvoyer l'indice du noeud

    Node n;
    BBox nodeBox;
  	nodeBox.pmin = min(nodes[left].bounds.pmin, nodes[right].bounds.pmin);
  	nodeBox.pmax = max(nodes[left].bounds.pmax, nodes[right].bounds.pmax);
    n.bounds = nodeBox;
    n.prime = -1;
    // n.prime = primitives[mid].triangle_id;
    n.filsg = left;
    n.filsd = right;

    nodes.push_back(n);
    return nodes.size() -1;
}



//! representation d'une source de lumiere.
struct Source : public Triangle
{
    Color emission;     //! flux emis.

    Source( ) : Triangle(), emission() {}
    Source( const TriangleData& data, const Color& color ) : Triangle(data), emission(color) {}
};

// ensemble de sources de lumieres
std::vector<Source> sources;
std::vector<float> cdf_sources;

// recuperer les sources de lumiere du mesh : triangles associee a une matiere qui emet de la lumiere, material.emission != 0
int build_sources( const Mesh& mesh )
{
    float cpt_cdf = 0;
    for(int i= 0; i < mesh.triangle_count(); i++)
    {
        // recupere la matiere associee a chaque triangle de l'objet
        Material material= mesh.triangle_material(i);

        if((material.emission.r + material.emission.g + material.emission.b) > 0)
        {
            // inserer la source de lumiere dans l'ensemble.
            sources.push_back( Source(mesh.triangle(i), material.emission) );

            // construire la CDF des sources en fonction des aires
            cdf_sources.push_back(cpt_cdf += sources[sources.size()-1].area());

        }
    }

    // normaliser la CDF
    for(unsigned int i = 0; i < cdf_sources.size(); i++)
    {
        cdf_sources[i] = cdf_sources[i] / cdf_sources[cdf_sources.size() - 1];
    }

    printf("%d sources.\n", (int) sources.size());
    return (int) sources.size();
}


// verifie que le rayon touche une source de lumiere.
bool direct( const Ray& ray )
{
    for(size_t i= 0; i < sources.size(); i++)
    {
        float t, u, v;
        if(sources[i].intersect(ray, ray.tmax, t, u, v))
            return true;
    }

    return false;
}

// recuperer les triangles du mesh
int build_triangles( const Mesh &mesh )
{
    for(int i= 0; i < mesh.triangle_count(); i++){
        triangles.push_back( Triangle(mesh.triangle(i)) );
      }

    printf("%d triangles.\n", (int) triangles.size());
    return (int) triangles.size();
}

int build_primitives( const std::vector<Triangle> & tri, std::vector<Primitive> & vecPrim){
    for(unsigned int i = 0; i < tri.size(); i++)
    {
        /*calcule des limites de la bbox*/
        Primitive p;
        p.bounds.pmin = min(min(Point(tri[i].a), Point(tri[i].b)), Point(tri[i].c));
        p.bounds.pmax = max(max(Point(tri[i].a), Point(tri[i].b)), Point(tri[i].c));
        p.center = Point(p.bounds.pmax.x + p.bounds.pmin.x/ 2.0f, p.bounds.pmax.y + p.bounds.pmin.y/ 2.0f, p.bounds.pmax.z + p.bounds.pmin.z/ 2.0f);
        p.triangle_id = i;

        vecPrim.push_back(p);
    }
    return (int) vecPrim.size();
}

// calcule l'intersection d'un rayon et de tous les triangles
bool intersect( const Ray& ray, Hit& hit )
{
    hit.t= ray.tmax;
    for(size_t i= 0; i < triangles.size(); i++)
    {
        float t, u, v;
        if(triangles[i].intersect(ray, hit.t, t, u, v))
        {
            hit.t= t;
            hit.u= u;
            hit.v= v;

            hit.p= ray(t);      // evalue la positon du point d'intersection sur le rayon
            hit.n= triangles[i].normal(u, v);

            hit.object_id= i;	// permet de retrouver toutes les infos associees au triangle
        }
    }

    return (hit.object_id != -1);
}

// calcule l'intersection d'un rayon et de toutes les sources
bool intersectlight( const Ray& ray, Hit& hit )
{
    hit.t= ray.tmax;
    for(size_t i= 0; i < sources.size(); i++)
    {
        float t, u, v;
        if(sources[i].intersect(ray, hit.t, t, u, v))
        {
            hit.t= t;
            hit.u= u;
            hit.v= v;

            hit.p= ray(t);      // evalue la positon du point d'intersection sur le rayon
            hit.n= sources[i].normal(u, v);

            hit.object_id= i;	// permet de retrouver toutes les infos associees au triangle
        }
    }

    return (hit.object_id != -1);
}

/* fonction d'intersection d'une node */
void intersectNode( const Ray& ray, Hit& hit, const int & idNode, const std::vector<Node>& nodes)
{
  // si je ne suis pas une feuille :
  if(nodes[idNode].prime == -1)
  {
      // je teste la boite du fils gauche
      float left_min = hit.t, left_max;
      bool left = nodes[idNode].filsg != -1 ? nodes[nodes[idNode].filsg].bounds.intersect(ray, hit.t, left_min, left_max) : false;
      // je teste la boite du fils droit
      float right_min = FLT_MAX, right_max;
      bool right = nodes[idNode].filsd != -1 ? nodes[nodes[idNode].filsd].bounds.intersect(ray, hit.t, right_min, right_max) : false;

      // on choisi quel fils regarder en premier
      if( left_min <= right_min)
      {
          if( left ) intersectNode(ray, hit, nodes[idNode].filsg, nodes);
          if( right && right_min <= hit.t ) intersectNode(ray, hit, nodes[idNode].filsd, nodes);
      }
      else if( right_min < left_min )
      {
          if( right ) intersectNode(ray, hit, nodes[idNode].filsd, nodes);
          if( left && left_min <= hit.t ) intersectNode(ray, hit, nodes[idNode].filsg, nodes);
      }
  }
  else
  {
      float t = FLT_MAX, u, v;
      // intersection avec le triangle
      if( triangles[nodes[idNode].prime].intersect(ray, hit.t, t, u, v) )
      {
          if( t < hit.t )
          {
              hit.t= t;
              hit.u= u;
              hit.v= v;
              hit.p= ray(t);      // evalue la positon du point d'intersection sur le rayon
              hit.n= triangles[nodes[idNode].prime].normal(u, v);
              hit.object_id= nodes[idNode].prime;	// permet de retrouver toutes les infos associees au triangle
          }
      }
  }
}

/* fonction qui lance l'appel recursif sur l'arbre */
bool intersectBVH( const Ray& ray, Hit& hit, const int & idNode, const std::vector<Node>& nodes)
{
    hit.t = ray.tmax;
    intersectNode(ray, hit, idNode, nodes);
    return hit.object_id != -1;
}

bool visibleBVH( const Ray& ray, Hit& hit, const int & idNode, const std::vector<Node>& nodes)
{
  // si je ne suis pas une feuille :
  if(nodes[idNode].prime == -1)
  {
      // je teste la boite du fils gauche
      float left_min = hit.t, left_max;
      bool left = nodes[idNode].filsg != -1 ? nodes[nodes[idNode].filsg].bounds.intersect(ray, hit.t, left_min, left_max) : false;
      // je teste la boite du fils droit
      float right_min = FLT_MAX, right_max;
      bool right = nodes[idNode].filsd != -1 ? nodes[nodes[idNode].filsd].bounds.intersect(ray, hit.t, right_min, right_max) : false;

      bool retd = true, retg = true;
      // on choisi quel fils regarder en premier
      if( left_min <= right_min)
      {
          if( left )
              retg = visibleBVH(ray, hit, nodes[idNode].filsg, nodes);
          if( right )
              retd = visibleBVH(ray, hit, nodes[idNode].filsd, nodes);
      }
      else if( right_min < left_min )
      {
          if( right )
              retd = visibleBVH(ray, hit, nodes[idNode].filsd, nodes);
          if( left )
              retg = visibleBVH(ray, hit, nodes[idNode].filsg, nodes);
      }
      return (retd && retg);
  }
  else
  {
      float t = FLT_MAX, u, v;
      // intersection avec le triangle
      if( triangles[nodes[idNode].prime].intersect(ray, hit.t, t, u, v) )
          return false;
  }
  return true;
}

// b1, b2, n sont 3 axes orthonormes.
void branchlessONB(const Vector &n, Vector &b1, Vector &b2)
{
    float sign = std::copysign(1.0f, n.z);
    const float a = -1.0f / (sign + n.z);
    const float b = n.x * n.y * a;
    b1 = Vector(1.0f + sign * n.x * n.x * a, sign * b, -sign * n.x);
    b2 = Vector(b, sign + n.y * n.y * a, -n.y);
}

/*****************************************************/
/*****************************************************/
/*****************************************************/

/* renvoie la couleur diffuse du material, avec ça couleur d'émission si
   c'est une source de lumière */
Color diffuse(const Ray& ray, Hit& hit, Material mat){
  float ls;
  if(dot(-(hit.p - ray.o),hit.n) > 0.0f){
    ls = dot(-(hit.p - ray.o),hit.n);
  }else{
    ls = 0.0f;
  }
  return mat.diffuse * ls + mat.emission;
}

/* calcule nb_point directions sur la spiral de fibonacci */
std::vector<Vector> fibonacci(int nb_point){
  std::vector<Vector> ret;
  for(int i = 1; i <= nb_point; i++){
    float cos_theta = 1.0 - ( 2.0 * i + 1.0)/( 2.0 * nb_point );
    float phi = (sqrt(5.0) + 1.0)/2.0;
    float phi_theta = 2.0 * PI * (float(i)/phi - floor(float(i)/phi));
    float sin_theta = sqrt(1.0 - cos_theta * cos_theta);

    Vector v(cos(phi_theta) * sin_theta, sin(phi_theta) * sin_theta, cos_theta);

    ret.push_back(v);
  }
  return ret;
}

/* calcule nb_point directions sur la spiral de fibonacci avec un bruit */
std::vector<Vector> fibonacci_disrupted(int nb_point){
  std::vector<Vector> ret;
  for(int i = 1; i <= nb_point; i++){
    float cos_theta = 1.0 - ( 2.0 * i + 1.0)/( 2.0 * nb_point );
    float phi = (sqrt(5.0) + 1.0)/2.0;
    float u = (rand() % 10000);
    u = u /10000;
    float phi_theta = 2.0 * PI * (float(i+u)/phi - floor(float(i+u)/phi));
    float sin_theta = sqrt(1.0 - cos_theta * cos_theta);

    Vector v(cos(phi_theta) * sin_theta, sin(phi_theta) * sin_theta, cos_theta);

    ret.push_back(v);
  }
  return ret;
}

/* transforme la spiral de fibonacci vers le repère monde */
std::vector<Vector> fibonacci_world(const Ray& ray, Hit& hit){
  Vector b1,b2;

  std::vector<Vector> ret;

  branchlessONB(hit.n, b1, b2);
  std::vector<Vector> fibo = fibonacci_disrupted(NB_FIBO_DIR);

  for(unsigned int i = 0; i<fibo.size(); i++)
  {
      Vector v = fibo[i].x * b1 + fibo[i].y * b2 + fibo[i].z * hit.n;
      ret.push_back(v);
  }
  return ret;
}

/* renvoie la enième directions de fibonacci */
Vector fibo( const int i, const int nb )
{
    float cos_theta = 1.0 - ( 2.0 * i + 1.0) / ( 2.0 * nb );
    float phi = (sqrt(5.0) + 1.0)/2.0;
    float u = float(rand() % 10000) / 10000.f;
    float phi_theta = 2.0 * PI * (float(i+u)/phi - floor(float(i+u)/phi));
    float sin_theta = sqrt(1.0 - cos_theta * cos_theta);
    return Vector(cos(phi_theta) * sin_theta, sin(phi_theta) * sin_theta, cos_theta);
}

/* transforme un vecteur vers le repère monde */
Vector l2w(const Vector& v, const Vector& n, const Vector& b1, const Vector& b2)
{
    return v.x * b1 + v.y * b2 + v.z * n;
}

bool isSource(const Material& mat)
{
    return (mat.emission.r + mat.emission.g + mat.emission.b) > 0;
}

int findDicho(const std::vector<float> & vec, const float & selec_random, const int & begin, const int & end)
{
    int mid = begin + (end - begin)/2;

    if(mid > begin && mid < end)
    {
        if(vec[mid] > selec_random)
        {
            return findDicho(vec, selec_random, begin, mid);
        }
        else if(vec[mid] < selec_random)
        {
            return findDicho(vec, selec_random, mid, end);
        }
        else
            return mid;
    }
    else
    {
        return vec[begin] > selec_random ? end : begin;
    }
}

// calcule la couleur directe
Color direct( const Point& p, const Vector& n, const Material& mat, const int & idNode, const std::vector<Node>& nodes)
{
    Color color;
    // V1 : selectionne une source uniformement
    // int id_random_source = rand() % sources.size();

    // V2 : selectionne une source en fonction de son aire
    float selec_random = uniform(gen);

    unsigned int id_random_source = 0;

    // recherche linéaire
    // while(id_random_source < cdf_sources.size() && cdf_sources[id_random_source] < selec_random) id_random_source++;

    // recherche dichotomique
    id_random_source = findDicho(cdf_sources, selec_random, 0, cdf_sources.size()-1);
    // selection de la source
    // std::cout << "id source : " << id_random_source << std::endl;
    Source source_rand = sources[id_random_source];

    Point s;
    // Selectionne un point uniforme sur la source selectionner
    // cf: Global Illumination Compendium - Dutre
    float u = sqrt(uniform(gen));
    float v = ( 1.0f - u ) * sqrt(uniform(gen));

    s = source_rand.point( u, v );
    // test de visibilite entre s et p

    Ray ray(p + n * EPSILON,s + source_rand.normal( u, v ) * EPSILON);
    Hit hit;
    hit.t = 1;

    // if(!intersectBVH(ray, hit, idNode, nodes) && ray.tmax <= hit.t)
    if(visibleBVH(ray, hit, idNode, nodes))
    {
        float cos_theta = std::max(0.0f, dot(n, ray.d));
        Color brdf = mat.diffuse / M_PI;
        Color le = source_rand.emission;
        color = le * brdf * cos_theta ;
    }

    return color;
}

Color indirect( const Point& p, const Vector& n, const Material& mat,
    const int & idNode, const std::vector<Node>& nodes,
    const Mesh & mesh, const int & i)
{
    Color color;
    Vector b1, b2;
    branchlessONB(n, b1, b2);

    Vector w = l2w(fibo(i, NB_FIBO_DIR), n, b1, b2);

    Ray ray(p + 0.001f * n, w);
    Hit hit;

    if(intersectBVH(ray, hit, idNode, nodes))
    {
        const Material& mat2 = mesh.triangle_material(hit.object_id);
        color = direct( hit.p, hit.n, mat2, idNode, nodes);
    }

    float cos_theta = std::max(0.0f, dot(n, ray.d));
    Color color2 = cos_theta * mat.diffuse;

    return color * color2;
}

/*****************************************************/
/*****************************************************/
/*****************************************************/

int main( int argc, char **argv )
{
    // init generateur aleatoire
    srand(time(NULL));

    // lire un maillage et ses matieres
    // Mesh mesh= read_mesh("data/cornell.obj");
    Mesh mesh= read_mesh("data/sanMiguel/san-miguel.obj");
    // Mesh mesh= read_mesh("data/theCabin.obj");
    // Mesh mesh= read_mesh("data/AttackDeathStar2.obj");
    // Mesh mesh= read_mesh("data/TheCarnival.obj");

    if(mesh == Mesh::error())
        return 1;

    // extraire les sources
    build_sources(mesh);
    // extraire les triangles du maillage
    build_triangles(mesh);

    // construit les primitives des sources et des triangles
    build_primitives(triangles, primitives);

    // relire une camera
    Orbiter camera;
    float fov = 50;

    // camera.read_orbiter("data/orbiter/orbiter_cornell.txt");
    camera.read_orbiter("data/orbiter/orbiter_sanmiguel.txt");
     // camera.read_orbiter("data/orbiter/orbiter_theCabin.txt");
    //  camera.read_orbiter("data/orbiter/orbiter_AttackDeathStar2.txt");
    //  camera.read_orbiter("data/orbiter/orbiter_TheCarnival.txt");

    // placer une source de lumiere
    Point o= camera.position();

    // creer l'image pour stocker le resultat
    Image image(1920, 1080);
    // Image image(1024, 640);

    std::vector<Node> nodes;

    int headBVH = build_node(nodes, primitives, 0, primitives.size());

    std::cout << "nodes " << nodes.size() << " leaves " << primitives.size() << " root " << headBVH << std::endl;

    int lines = 0;
// multi thread avec OpenMP
#pragma omp parallel for schedule(dynamic, 16)
    for(int y= 0; y < image.height(); y++)
    {
    #pragma omp atomic
        lines++;
        if( lines % 16 == 0 )
            std::cout << float(lines) * 100.f / float(image.height()) << std::endl;

        for(int x= 0; x < image.width(); x++)
        {
            // Point o= light;	// origine du rayon
            Color sample;
            // Calcule de l'anti aliasing
            for( int index = 0; index < NB_SAMPLE; ++index )
            {
                Point d1;
                Vector dx1, dy1;
                camera.frame(image.width(), image.height(), 1, fov, d1, dx1, dy1);
                float rx = uniform(gen)-0.5, ry = uniform(gen)-0.5;
                Point e= d1 + (x + rx) * dx1 + ( y + ry ) * dy1;

                Ray ray(o, e);
                Hit hit;
                if(intersectBVH(ray, hit, headBVH, nodes))
                {
                    const Material& mat = mesh.triangle_material(hit.object_id);
                    if(mat.emission.power() > 0)
                    {
                        sample = sample + mat.emission ;
                    }
                    else
                    {
                            sample = sample + direct( hit.p, hit.n, mat, headBVH, nodes) / (float)NB_SAMPLE +
                                indirect(hit.p, hit.n, mat, headBVH, nodes, mesh, index) / (float)NB_SAMPLE;
                    }
                }
            }
            image(x, y)= Color(sample, 1);
        }
    }
    write_image(image, "render.png");
    write_image_hdr(image, "render.hdr");
    return 0;
}
