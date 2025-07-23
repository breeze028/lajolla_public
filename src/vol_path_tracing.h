#pragma once

#include "scene.h"
#include "pcg.h"
#include "transform.h"

// The simplest volumetric renderer: 
// single absorption only homogeneous volume
// only handle directly visible light sources
Spectrum vol_path_tracing_1(const Scene &scene,
                            int x, int y, /* pixel coordinates */
                            pcg32_state &rng) {
    // Homework 2: implememt this!
    int w = scene.camera.width, h = scene.camera.height;
    Vector2 screen_pos((x + next_pcg32_real<Real>(rng)) / w,
                       (y + next_pcg32_real<Real>(rng)) / h);
    Ray ray = sample_primary(scene.camera, screen_pos);
    // For this homework, we will not use ray differentials.
    RayDifferential ray_diff = RayDifferential{Real(0), Real(0)};

    std::optional<PathVertex> vertex_ = intersect(scene, ray, ray_diff);
    if (!vertex_) {
        // Hit background.
        return make_zero_spectrum();
    }
    PathVertex vertex = *vertex_;

    Spectrum radiance = make_zero_spectrum();

    // Compute the transmittance through the medium.
    int medium_id = vertex.exterior_medium_id;
    Medium medium = scene.media[medium_id];
    Spectrum sigma_a = get_sigma_a(medium, vertex.position);
    Real t_hit = distance<double>(vertex.position, ray.org); 
    Spectrum transmittance = exp(-sigma_a * t_hit);
    
    if (is_light(scene.shapes[vertex.shape_id])) {
        // We hit a light immediately.
        radiance = transmittance * emission(vertex, -ray.dir, scene);
        return radiance;
    }
    return radiance;
}

// The second simplest volumetric renderer: 
// single monochromatic homogeneous volume with single scattering,
// no need to handle surface lighting, only directly visible light source
// Spectrum vol_path_tracing_2(const Scene &scene,
//                             int x, int y, /* pixel coordinates */
//                             pcg32_state &rng) {
//     // Homework 2: implememt this!
//     // TODO: need accounting for non hit ray
//     int w = scene.camera.width, h = scene.camera.height;
//     Vector2 screen_pos((x + next_pcg32_real<Real>(rng)) / w,
//                        (y + next_pcg32_real<Real>(rng)) / h);
//     Ray ray = sample_primary(scene.camera, screen_pos);
//     // For this homework, we will not use ray differentials.
//     RayDifferential ray_diff = RayDifferential{Real(0), Real(0)};

//     std::optional<PathVertex> vertex_ = intersect(scene, ray, ray_diff);

//     Spectrum radiance = make_zero_spectrum();

//     // Compute the transmittance through the medium.
//     int medium_id = scene.camera.medium_id;
//     Medium medium = scene.media[medium_id];
//     Spectrum sigma_a = get_sigma_a(medium, ray.org);
//     Spectrum sigma_s = get_sigma_s(medium, ray.org);
//     Spectrum sigma_t = sigma_a + sigma_s;
//     Real t_hit = 0;
//     if (vertex_) {
//         t_hit = distance<double>(vertex_->position, ray.org);
//     } 
//     Spectrum transmittance = exp(-sigma_t * t_hit);

//     Real u = next_pcg32_real<Real>(rng);
//     Real t = -log(1 - u) / sigma_t.x; // The volume is monochromatic.

//     if (t < t_hit || !vertex_) {
//         Real trans_pdf = exp(-sigma_t.x * t) * sigma_t.x;
//         Spectrum transmittance = exp(-sigma_t * t);
        
//         // Compute L_s1 using Monte Carlo sampling.
//         Vector3 p = ray.org + t * ray.dir;
//         // Sample the light.
//         Vector2 light_uv{next_pcg32_real<Real>(rng), next_pcg32_real<Real>(rng)};
//         Real shape_w = next_pcg32_real<Real>(rng);
//         int light_id = 0;
//         const Light& light = scene.lights[light_id];
//         PointAndNormal point_on_light = 
//             sample_point_on_light(light, p, light_uv, shape_w, scene);
//         Vector3 dir_light = normalize(point_on_light.position - p);
//         Ray shadow_ray{p, dir_light, 
//                         get_shadow_epsilon(scene),
//                         (1 - get_shadow_epsilon(scene)) *
//                         distance(point_on_light.position, p)};
//         Real G = 0;
//         if (!occluded(scene, shadow_ray)) {
//             // If the point on light is occluded, G is 0.
//             G = max(-dot(dir_light, point_on_light.normal), Real(0)) /
//                 distance_squared(point_on_light.position, p);
//         }

//         Real L_s1_pdf = pdf_point_on_light(light, point_on_light, p, scene);

//         Spectrum L_s1 = make_zero_spectrum();
//         if (G > 0 && L_s1_pdf > 0) {
//             Vector3 dir_view = -ray.dir;
//             PhaseFunction phase = get_phase_function(medium);
//             Spectrum f = eval(phase, dir_view, dir_light);
//             Spectrum transmittance = exp(-sigma_t * distance<double>(p, point_on_light.position));
//             Spectrum L_e = emission(light, -dir_light, Real(0), point_on_light, scene);
//             L_s1 = transmittance * G * f * L_e / L_s1_pdf;
//         }
//         radiance = (transmittance / trans_pdf) * sigma_s * L_s1;
//         return radiance;
//     } else {
//         // Hit a surface, account for surface emission.
//         Real trans_pdf = exp(-sigma_t.x * t_hit);
//         Spectrum transmittance = exp(-sigma_t * t_hit);
//         PathVertex vertex = *vertex_;
//         if (is_light(scene.shapes[vertex.shape_id])) {
//             // We hit a light immediately.
//             radiance += transmittance * emission(vertex, -ray.dir, scene) / trans_pdf;
//             return radiance;
//         }
//     }
// }

Spectrum vol_path_tracing_2(const Scene &scene,
                            int x, int y, /* pixel coordinates */
                            pcg32_state &rng) {
    // Homework 2: implememt this!
    int w = scene.camera.width, h = scene.camera.height;
    Vector2 screen_pos((x + next_pcg32_real<Real>(rng)) / w,
                       (y + next_pcg32_real<Real>(rng)) / h);
    Ray ray = sample_primary(scene.camera, screen_pos);
    RayDifferential ray_diff = RayDifferential{Real(0), Real(0)};
    std::optional<PathVertex> vertex_ = intersect(scene, ray, ray_diff);
    
    Real t_hit = infinity<Real>();
    Medium medium = scene.media[scene.camera.medium_id];
    PathVertex vertex;
    if (vertex_) {
        vertex = *vertex_;
        int medium_id = dot(vertex.shading_frame.n, vertex.geometric_normal) < 0 ? vertex.interior_medium_id : vertex.exterior_medium_id;
        medium = scene.media[medium_id];
        t_hit = distance(vertex.position, ray.org);
    }
    
    // since homogeneous medium, doesnot care about position
    Spectrum sigma_a = get_sigma_a(medium, Vector3(0, 0, 0));
    Spectrum sigma_s = get_sigma_s(medium, Vector3(0, 0, 0));
    Spectrum sigma_t = sigma_a + sigma_s;

    Real t = -log(1 - next_pcg32_real<Real>(rng)) / sigma_t[0];
    if (t >= t_hit) {
        // hit surface, account for emission
        if(!is_light(scene.shapes[vertex.shape_id])) {
            return make_zero_spectrum();
        }
        // where (transmittance / transmittance_pdf) 
        //          = exp(-sigma_t * t_hit) / exp(-sigma_t * t_hit) = 1
        return emission(vertex, -ray.dir, scene);
    }
    else{
        // hit volume, account for single scattering
        Vector3 volume_point = ray.org + t * ray.dir;

        // sample a light source
        Vector2 light_uv{next_pcg32_real<Real>(rng), next_pcg32_real<Real>(rng)};
        Real light_w = next_pcg32_real<Real>(rng);
        Real shape_w = next_pcg32_real<Real>(rng);
        int light_id = sample_light(scene, light_w);
        const Light &light = scene.lights[light_id];
        PointAndNormal point_on_light =
            sample_point_on_light(light, volume_point, light_uv, shape_w, scene);
        

        Spectrum L_distanceSampling = make_zero_spectrum();
        Real p_distanceSampling = 0;
        {
            // compute G, p1, L, light_transmittance
            Real t_light = distance(point_on_light.position, volume_point);
            Vector3 dir_light = normalize(point_on_light.position - volume_point);
            
            Real G = 0;
            Ray shadow_ray{volume_point, dir_light, get_shadow_epsilon(scene), t_light - get_shadow_epsilon(scene)};
            if (!occluded(scene, shadow_ray))
                G = max(abs(dot(dir_light, point_on_light.normal)), Real(0)) / (t_light * t_light);

            Real p_light = light_pmf(scene, light_id) *
                pdf_point_on_light(light, point_on_light, volume_point, scene);

            Spectrum L = Spectrum(0, 0, 0);
            if(p_light > 0 && G > 0)
                L = emission(light, -dir_light, Real(0), point_on_light, scene);

            PhaseFunction& phase_function = get_phase_function(medium);
            Spectrum rho = eval(phase_function, -ray.dir, dir_light);
            
            Spectrum light_transmittance = exp(-t_light * sigma_t);

            // monochromatic medium
            Real sigma_ts = sigma_t[0];

            L_distanceSampling = exp(-t * sigma_ts)* rho * G * light_transmittance * L;
            p_distanceSampling = p_light * exp(-t * sigma_ts) * sigma_ts;
        }
        // return sigma_s * L_distanceSampling / p_distanceSampling;

        // MIS with equiangular sampling
        Spectrum L_equiangularSampling = make_zero_spectrum();
        Real p_equiangularSampling = 0;
        {
            Vector3 dir_a = point_on_light.position - ray.org;
    
            Real a = dot(dir_a, ray.dir) / length_squared(ray.dir);
            Real b = t_hit - a;
            Real D = length(ray.dir * a + ray.org - point_on_light.position);

            Real theta_a = atan(a / D);
            Real theta_b = atan(b / D);

            Real u_rand = next_pcg32_real<Real>(rng);
            t = a + D * tan((theta_a  + theta_b) * u_rand - theta_a);

            volume_point = ray.org + t * ray.dir;

            // compute G, p, L, light_transmittance
            Real t_light = distance(point_on_light.position, volume_point);
            Vector3 dir_light = normalize(point_on_light.position - volume_point);
            
            Real G = 0;
            Ray shadow_ray{volume_point, dir_light, get_shadow_epsilon(scene), t_light - get_shadow_epsilon(scene)};
            if (!occluded(scene, shadow_ray))
                G = max(abs(dot(dir_light, point_on_light.normal)), Real(0)) / (t_light * t_light);

            Real p_light = light_pmf(scene, light_id) *
                pdf_point_on_light(light, point_on_light, volume_point, scene);

            Spectrum L = Spectrum(0, 0, 0);
            if(p_light > 0 && G > 0)
                L = emission(light, -dir_light, Real(0), point_on_light, scene);

            PhaseFunction& phase_function = get_phase_function(medium);
            Spectrum rho = eval(phase_function, -ray.dir, dir_light);
            
            Spectrum light_transmittance = exp(-t_light * sigma_t);

            // monochromatic medium
            Real sigma_ts = sigma_t[0];

            L_equiangularSampling = exp(-t * sigma_ts)* rho * G * light_transmittance * L;
            p_equiangularSampling = p_light * D / ((theta_a + theta_b) * (D*D + t_light * t_light));
        }
        // return sigma_s * L_equiangularSampling / p_equiangularSampling;

        Real p_sum = p_distanceSampling + p_equiangularSampling;
        if(p_sum == 0)
            return make_zero_spectrum();
        
        return sigma_s * (L_distanceSampling + L_equiangularSampling) / p_sum;
    }
}


// The third volumetric renderer (not so simple anymore): 
// multiple monochromatic homogeneous volumes with multiple scattering
// no need to handle surface lighting, only directly visible light source
Spectrum vol_path_tracing_3(const Scene &scene,
                            int x, int y, /* pixel coordinates */
                            pcg32_state &rng) {
    // Homework 2: implememt this!
    return make_zero_spectrum();
}

// The fourth volumetric renderer: 
// multiple monochromatic homogeneous volumes with multiple scattering
// with MIS between next event estimation and phase function sampling
// still no surface lighting
Spectrum vol_path_tracing_4(const Scene &scene,
                            int x, int y, /* pixel coordinates */
                            pcg32_state &rng) {
    // Homework 2: implememt this!
    return make_zero_spectrum();
}

// The fifth volumetric renderer: 
// multiple monochromatic homogeneous volumes with multiple scattering
// with MIS between next event estimation and phase function sampling
// with surface lighting
Spectrum vol_path_tracing_5(const Scene &scene,
                            int x, int y, /* pixel coordinates */
                            pcg32_state &rng) {
    // Homework 2: implememt this!
    return make_zero_spectrum();
}

// The final volumetric renderer: 
// multiple chromatic heterogeneous volumes with multiple scattering
// with MIS between next event estimation and phase function sampling
// with surface lighting
Spectrum vol_path_tracing(const Scene &scene,
                          int x, int y, /* pixel coordinates */
                          pcg32_state &rng) {
    // Homework 2: implememt this!
    return make_zero_spectrum();
}
