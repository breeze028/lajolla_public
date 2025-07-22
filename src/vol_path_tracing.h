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
Spectrum vol_path_tracing_2(const Scene &scene,
                            int x, int y, /* pixel coordinates */
                            pcg32_state &rng) {
    // Homework 2: implememt this!
    // TODO: need accounting for non hit ray
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
    Spectrum sigma_s = get_sigma_s(medium, vertex.position);
    Spectrum sigma_t = sigma_a + sigma_s;
    Real t_hit = distance<double>(vertex.position, ray.org); 
    Spectrum transmittance = exp(-sigma_t * t_hit);

    Real u = next_pcg32_real<Real>(rng);
    Real t = -log(1 - u) / sigma_t.x; // The volume is monochromatic.

    if (t < t_hit) {
        Real trans_pdf = exp(-sigma_t.x * t) * sigma_t.x;
        Spectrum transmittance = exp(-sigma_t * t);
        
        // Compute L_s1 using Monte Carlo sampling.
        Vector3 p = ray.org + t * ray.dir;
        // Sample the light.
        Vector2 light_uv{next_pcg32_real<Real>(rng), next_pcg32_real<Real>(rng)};
        Real shape_w = next_pcg32_real<Real>(rng);
        int light_id = 0;
        const Light& light = scene.lights[light_id];
        PointAndNormal point_on_light = 
            sample_point_on_light(light, p, light_uv, shape_w, scene);
        Vector3 dir_light = normalize(point_on_light.position - p);
        Ray shadow_ray{p, dir_light, 
                        get_shadow_epsilon(scene),
                        (1 - get_shadow_epsilon(scene)) *
                        distance(point_on_light.position, p)};
        Real G = 0;
        if (!occluded(scene, shadow_ray)) {
            // If the point on light is occluded, G is 0.
            G = max(-dot(dir_light, point_on_light.normal), Real(0)) /
                distance_squared(point_on_light.position, p);
        }

        Real L_s1_pdf = pdf_point_on_light(light, point_on_light, p, scene);

        Spectrum L_s1 = make_zero_spectrum();
        if (G > 0 && L_s1_pdf > 0) {
            Vector3 dir_view = -ray.dir;
            PhaseFunction phase = get_phase_function(medium);
            Spectrum f = eval(phase, dir_view, dir_light);
            Spectrum transmittance = exp(-sigma_t * distance<double>(p, point_on_light.position));
            Spectrum L_e = emission(light, -dir_light, Real(0), point_on_light, scene);
            L_s1 = transmittance * G * f * L_e / L_s1_pdf;
        }
        radiance = (transmittance / trans_pdf) * sigma_s * L_s1;
        return radiance;
    } else {
        // Hit a surface, account for surface emission.
        Real trans_pdf = exp(-sigma_t.x * t_hit);
        Spectrum transmittance = exp(-sigma_t * t_hit);
        if (is_light(scene.shapes[vertex.shape_id])) {
            // We hit a light immediately.
            radiance += transmittance * emission(vertex, -ray.dir, scene);
            return radiance;
        }
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
