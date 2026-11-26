# C++ test -> Rust counterpart map

Every gtest case in the C++ tree at `6baf46a` is listed below once. A case is
`ported` when a Rust test asserts the same behaviour, `exempt` when the case is
inherently about the EGL/OSG/wgpu-independent implementation (reasons below),
and `deferred` is empty: the two windowed cases are exempt because they need a display.

## Counting method

`grep -rE '^\s*(TEST|TEST_F|TEST_P)\(' tests --include=*.cpp | wc -l` finds 208 declarations.
One of them is `TEST_P(BowlModelBakeParamTest, BakedSlots_MatchCameraCount)`
instantiated for N4/N6/N8, so the runtime case count is 208 - 1 + 3 = **210**.

`ctest -N` in the pre-existing `build/` reports 215, but that build was configured
from a different revision: it contains five `Camera.*Name` cases absent from
`6baf46a:tests/camera_model/test_camera.cpp` (which has three cases). The
source count above is the one this document uses.

## Summary

| C++ file | cases | ported | exempt | deferred |
| --- | ---: | ---: | ---: | ---: |
| `tests/bowl_model/test_blend_weights.cpp` | 18 | 18 | 0 | 0 |
| `tests/bowl_model/test_bowl_model.cpp` | 34 | 34 | 0 | 0 |
| `tests/bowl_model/test_bowl_params.cpp` | 20 | 19 | 1 | 0 |
| `tests/camera_model/test_camera.cpp` | 3 | 3 | 0 | 0 |
| `tests/camera_model/test_camera_extrinsics.cpp` | 12 | 12 | 0 | 0 |
| `tests/camera_model/test_camera_model.cpp` | 4 | 4 | 0 | 0 |
| `tests/camera_model/test_kb_camera_model.cpp` | 21 | 21 | 0 | 0 |
| `tests/camera_model/test_lut_camera_model.cpp` | 10 | 10 | 0 | 0 |
| `tests/camera_model/test_ocam_camera_model.cpp` | 7 | 7 | 0 | 0 |
| `tests/common/test_angle_utils.cpp` | 8 | 8 | 0 | 0 |
| `tests/core/test_stitch_engine.cpp` | 22 | 22 | 0 | 0 |
| `tests/depth/test_ground_depth.cpp` | 7 | 7 | 0 | 0 |
| `tests/depth/test_spherical_rectification.cpp` | 7 | 7 | 0 | 0 |
| `tests/render/test_osg_view_renderer.cpp` | 18 | 15 | 3 | 0 |
| `tests/render/test_stitch_scene.cpp` | 15 | 11 | 4 | 0 |
| `tests/render/test_egl_headless_context.cpp` | 4 | 0 | 4 | 0 |
| **total** | **210** | **198** | **12** | **0** |

Self-check: `210 == ported + exempt + deferred` and every `ported` counterpart
name resolves in `cargo test --workspace -- --list` (the check is repeated in
the commit report).

## Exemption reasons

1. **EGL/OSG implementation detail** - the case asserts an OSG object, an
   `osg::StateSet`, or an EGL context that wgpu does not model.
2. **Equivalent duplicate** - another Rust test asserts the same values, usually
   more strictly, so a second Rust test would add nothing.
3. **Implementation-detail assertion** - the case inspects `osg::Geometry` /
   `osg::Texture2DArray` internals rather than observable output.
4. **Display-dependent construction** - the case opens an interactive window,
   which the headless test environment cannot do; the camera and interaction
   math it depends on is covered by the `avm-viewer` controller tests.

## `tests/bowl_model/test_blend_weights.cpp`

| C++ file | C++ case | Rust counterpart | Status |
| --- | --- | --- | --- |
| `test_blend_weights.cpp` | `DefaultPole_BehaviorUnchanged` | `avm-core/tests/blend_weights.rs::default_pole_matches_explicit_zero_pole` | ported |
| `test_blend_weights.cpp` | `NonDefaultPole_ShiftsOrigin` | `avm-core/tests/blend_weights.rs::non_default_pole_shifts_the_origin` | ported |
| `test_blend_weights.cpp` | `N4Standard_Front` | `avm-core/tests/blend_weights.rs::four_standard_cameras_select_the_front_camera` | ported |
| `test_blend_weights.cpp` | `N4Standard_Rear` | `avm-core/tests/blend_weights.rs::four_standard_cameras_select_the_rear_camera` | ported |
| `test_blend_weights.cpp` | `N4Standard_BlendBandWidth` | `avm-core/tests/blend_weights.rs::four_camera_blend_half_angle_is_13_degrees` | ported |
| `test_blend_weights.cpp` | `N4_BlendZone` | `avm-core/tests/blend_weights.rs::four_camera_diagonal_blends_two_cameras_to_one` | ported |
| `test_blend_weights.cpp` | `N4_CosSmoothAtBoundary` | `avm-core/tests/blend_weights.rs::four_camera_cosine_smoothstep_is_half_half_on_the_boundary` | ported |
| `test_blend_weights.cpp` | `N4_OutsideBlendBand` | `avm-core/tests/blend_weights.rs::four_camera_point_outside_the_blend_band_has_a_single_owner` | ported |
| `test_blend_weights.cpp` | `N8_BlendHalfAngle` | `avm-core/tests/blend_weights.rs::eight_camera_blend_half_angle_is_6_5_degrees` | ported |
| `test_blend_weights.cpp` | `N8_WeightsSumToOne` | `avm-core/tests/blend_weights.rs::eight_camera_weights_sum_to_one_everywhere` | ported |
| `test_blend_weights.cpp` | `N3_ArbitrarySpacing` | `avm-core/tests/blend_weights.rs::three_camera_arbitrary_spacing_weights_sum_to_one` | ported |
| `test_blend_weights.cpp` | `AllWeightsNonNegative` | `avm-core/tests/blend_weights.rs::all_weights_are_non_negative` | ported |
| `test_blend_weights.cpp` | `N1_SingleCamera` | `avm-core/tests/blend_weights.rs::single_camera_always_has_full_weight` | ported |
| `test_blend_weights.cpp` | `EmptyCameras_ReturnsEmpty` | `avm-core/tests/blend_weights.rs::empty_camera_list_returns_empty_weights` | ported |
| `test_blend_weights.cpp` | `NormalizeWeights_ScalesBlendWeightsByEdgeFactor` | `avm-core/tests/blend_weights.rs::normalize_weights_scales_blend_weights_by_edge_factor` | ported |
| `test_blend_weights.cpp` | `NormalizeWeights_ZeroBlendWeightsFallBackToEdgeFactors` | `avm-core/tests/blend_weights.rs::normalize_weights_falls_back_to_edge_factors_when_blend_is_zero` | ported |
| `test_blend_weights.cpp` | `NormalizeWeights_GatedCamera_DropsItsShare` | `avm-core/tests/blend_weights.rs::normalize_weights_drops_a_gated_camera_share` | ported |
| `test_blend_weights.cpp` | `NormalizeWeights_NoVisibleCamera_LeavesWeightsUntouched` | `avm-core/tests/blend_weights.rs::normalize_weights_leaves_output_untouched_without_visible_camera` | ported |

## `tests/bowl_model/test_bowl_model.cpp`

| C++ file | C++ case | Rust counterpart | Status |
| --- | --- | --- | --- |
| `test_bowl_model.cpp` | `CarCenter_FlatRegionCenteredAtCarCenter` | `avm-core/tests/bowl_model.rs::car_center_flat_region_is_centred_at_the_car_center` | ported |
| `test_bowl_model.cpp` | `CarCenter_GridExtentSymmetricAboutCarCenter` | `avm-core/tests/bowl_model.rs::car_center_grid_extent_is_symmetric_about_the_car_center` | ported |
| `test_bowl_model.cpp` | `CarCenter_DefaultUnchanged` | `avm-core/tests/bowl_model.rs::car_center_default_is_unchanged` | ported |
| `test_bowl_model.cpp` | `ReturnsNonEmptyMesh` | `avm-core/tests/bowl_model.rs::returns_a_non_empty_mesh` | ported |
| `test_bowl_model.cpp` | `GridSizeMatchesParams` | `avm-core/tests/bowl_model.rs::grid_size_matches_params` | ported |
| `test_bowl_model.cpp` | `VerticesInXyBounds` | `avm-core/tests/bowl_model.rs::vertices_stay_inside_the_xy_bounds` | ported |
| `test_bowl_model.cpp` | `VerticesZInRange` | `avm-core/tests/bowl_model.rs::vertices_z_stay_within_max_height` | ported |
| `test_bowl_model.cpp` | `FlatCenterRegion` | `avm-core/tests/bowl_model.rs::flat_center_region_has_zero_height` | ported |
| `test_bowl_model.cpp` | `EdgesMonotonic` | `avm-core/tests/bowl_model.rs::height_is_monotonic_in_the_distance_from_the_platform` | ported |
| `test_bowl_model.cpp` | `NormalsAreUnitLength` | `avm-core/tests/bowl_model.rs::normals_are_unit_length` | ported |
| `test_bowl_model.cpp` | `FlatCenterNormalsUpward` | `avm-core/tests/bowl_model.rs::flat_center_normals_point_upward` | ported |
| `test_bowl_model.cpp` | `TrianglesHaveValidIndices` | `avm-core/tests/bowl_model.rs::triangles_have_valid_indices` | ported |
| `test_bowl_model.cpp` | `DifferentCarSizeChangesFlatRegion` | `avm-core/tests/bowl_model.rs::different_car_size_changes_the_flat_region` | ported |
| `test_bowl_model.cpp` | `MinimalParams_OneCellMesh` | `avm-core/tests/bowl_model.rs::minimal_params_still_build_a_one_cell_mesh` | ported |
| `test_bowl_model.cpp` | `BakeVertices_OneBakedPerMeshVertex` | `avm-core/tests/bowl_model.rs::bake_produces_one_baked_vertex_per_mesh_vertex` | ported |
| `test_bowl_model.cpp` | `SingleCamera_UVMatchesProjection` | `avm-core/tests/bowl_model.rs::single_camera_uv_matches_the_projection` | ported |
| `test_bowl_model.cpp` | `BakedSlots_MatchCameraCount/N4` | `avm-core/tests/bowl_model.rs::baked_slots_match_camera_count_for_4_6_and_8` | ported |
| `test_bowl_model.cpp` | `BakedSlots_MatchCameraCount/N6` | `avm-core/tests/bowl_model.rs::baked_slots_match_camera_count_for_4_6_and_8` | ported |
| `test_bowl_model.cpp` | `BakedSlots_MatchCameraCount/N8` | `avm-core/tests/bowl_model.rs::baked_slots_match_camera_count_for_4_6_and_8` | ported |
| `test_bowl_model.cpp` | `BakedVertex_SizedByCameraCount` | `avm-core/tests/bowl_model.rs::baked_vertex_is_sized_by_the_camera_count` | ported |
| `test_bowl_model.cpp` | `N4_WeightsSumToOne_WhenVisible` | `avm-core/tests/bowl_model.rs::four_camera_weights_sum_to_one_when_visible` | ported |
| `test_bowl_model.cpp` | `N4_UVInNormalizedRange` | `avm-core/tests/bowl_model.rs::four_camera_uvs_stay_in_the_normalized_range` | ported |
| `test_bowl_model.cpp` | `N4_InvisibleCameraHasZeroWeight` | `avm-core/tests/bowl_model.rs::four_camera_invisible_pair_has_zero_weight` | ported |
| `test_bowl_model.cpp` | `N4_AllVerticesHaveAtLeastOneVisible` | `avm-core/tests/bowl_model.rs::four_camera_bake_has_both_visible_and_invisible_vertices` | ported |
| `test_bowl_model.cpp` | `PointInRectButOutsideFisheyeCircle_Invisible` | `avm-core/tests/bowl_model.rs::point_in_rect_but_outside_the_fisheye_circle_is_invisible` | ported |
| `test_bowl_model.cpp` | `AllInvisible_WeightsStayZero_HasVisibleVerticesFalse` | `avm-core/tests/bowl_model.rs::all_invisible_vertices_keep_zero_weight_and_report_no_visibility` | ported |
| `test_bowl_model.cpp` | `SingleCamera_VisibleVerticesNormalizeToWeightOne` | `avm-core/tests/bowl_model.rs::single_camera_visible_vertices_normalize_to_weight_one` | ported |
| `test_bowl_model.cpp` | `EdgeGate_UnsafeCameraZero_SafeCameraGetsFullWeight` | `avm-core/tests/bowl_model.rs::edge_gate_unsafe_camera_is_dropped_while_safe_camera_gets_full_weight` | ported |
| `test_bowl_model.cpp` | `CarCenter_ShiftsBlendWeightDistribution` | `avm-core/tests/bowl_model.rs::car_center_shifts_the_blend_weight_distribution` | ported |
| `test_bowl_model.cpp` | `HasVisibleVertices_TrueWhenRimVisible` | `avm-core/tests/bowl_model.rs::has_visible_vertices_is_true_when_the_rim_is_visible` | ported |
| `test_bowl_model.cpp` | `D3_FallbackNormalization_OwnerInvisibleNeighborVisible` | `avm-core/tests/bowl_model.rs::fallback_normalization_gives_weight_to_a_visible_neighbour_only` | ported |
| `test_bowl_model.cpp` | `BelowCameraVertexStillGetsWeight` | `avm-core/tests/bowl_model.rs::vertex_below_the_camera_still_gets_weight` | ported |
| `test_bowl_model.cpp` | `EdgeGate_AllCamerasUnsafe_ZeroWeight` | `avm-core/tests/bowl_model.rs::edge_gate_zeroes_weight_when_all_cameras_are_unsafe` | ported |
| `test_bowl_model.cpp` | `LutCamera_BakesVisibleUv` | `avm-core/tests/bowl_model.rs::lut_camera_bakes_visible_uv` | ported |

## `tests/bowl_model/test_bowl_params.cpp`

| C++ file | C++ case | Rust counterpart | Status |
| --- | --- | --- | --- |
| `test_bowl_params.cpp` | `AccessorsRoundTrip` | `avm-core/tests/bowl_params.rs::accessors_round_trip` | ported |
| `test_bowl_params.cpp` | `FlatRegionHalfExtents` | `avm-core/tests/bowl_params.rs::flat_region_half_extents_are_car_plus_platform` | ported |
| `test_bowl_params.cpp` | `RdMatchesHandComputed` | `avm-core/tests/bowl_params.rs::distance_from_flat_platform_matches_hand_computation` | ported |
| `test_bowl_params.cpp` | `BowlZValues` | `avm-core/tests/bowl_params.rs::bowl_height_uses_the_power_law` | ported |
| `test_bowl_params.cpp` | `GridSegments` | `avm-core/tests/bowl_params.rs::grid_segments_rounds_the_radius_over_the_step` | ported |
| `test_bowl_params.cpp` | `FullCtorOverrideKeepsOriginalIntact` | `avm-core/tests/bowl_params.rs::full_constructor_override_keeps_the_original_intact` | ported |
| `test_bowl_params.cpp` | `ForCarSizeScaling` | `avm-core/tests/bowl_params.rs::for_car_size_scales_lengths_linearly_and_slope_inversely` | ported |
| `test_bowl_params.cpp` | `GenerateBlockBoatMesh_MeshStructure` | `avm-core/tests/bowl_params.rs::block_boat_mesh_structure_is_consistent` | ported |
| `test_bowl_params.cpp` | `GenerateBlockBoatMesh_CircleClip` | `avm-core/tests/bowl_params.rs::block_boat_mesh_clips_to_the_outer_radius` | ported |
| `test_bowl_params.cpp` | `GenerateBlockBoatMesh_MatchesBowlModelConstructorPath` | `avm-core/tests/bowl_params.rs::block_boat_mesh_matches_the_bowl_model_constructor_path` | ported |
| `test_bowl_params.cpp` | `CarScale_MatchesCalibratedValues` | ② 与 `car_anchor_is_exact_regression_to_old_default` 等价重复：同一组常量，后者用 `assert_eq!` 更严格 | exempt |
| `test_bowl_params.cpp` | `CarAnchor_ExactRegressionToOldDefault` | `avm-core/tests/bowl_params.rs::car_anchor_is_exact_regression_to_old_default` | ported |
| `test_bowl_params.cpp` | `LinearSimilarity_DoublingLengthDoublesAllLengths` | `avm-core/tests/bowl_params.rs::linear_similarity_doubling_length_doubles_all_lengths` | ported |
| `test_bowl_params.cpp` | `SelfSimilarity_OpeningRatioConstantAcrossScales` | `avm-core/tests/bowl_params.rs::self_similarity_opening_ratio_is_constant_across_scales` | ported |
| `test_bowl_params.cpp` | `IsotropicRimHeight_LongitudinalVsLateral` | `avm-core/tests/bowl_params.rs::isotropic_rim_height_stays_within_15_percent` | ported |
| `test_bowl_params.cpp` | `ShipScale_CoversCameras` | `avm-core/tests/bowl_params.rs::ship_scale_covers_the_camera_ring` | ported |
| `test_bowl_params.cpp` | `ShipScale_All8CamerasSeeBowlWithWeight` | `avm-core/tests/bowl_params.rs::ship_scale_all_eight_cameras_see_bowl_with_weight` | ported |
| `test_bowl_params.cpp` | `GridTopology_Constant126x126AcrossScales` | `avm-core/tests/bowl_params.rs::grid_topology_stays_126_by_126_across_scales` | ported |
| `test_bowl_params.cpp` | `DefaultCarCenter_BackwardCompatVertexCount` | `avm-core/tests/bowl_params.rs::default_params_bake_a_single_vertex_and_no_triangles` | ported |
| `test_bowl_params.cpp` | `ComputeRd_CarCenterShiftsFlatRegion` | `avm-core/tests/bowl_params.rs::car_center_shifts_the_flat_region` | ported |

## `tests/camera_model/test_camera.cpp`

| C++ file | C++ case | Rust counterpart | Status |
| --- | --- | --- | --- |
| `test_camera.cpp` | `DefaultIsInvalid` | `avm-core/tests/camera.rs::default_camera_is_invalid` | ported |
| `test_camera.cpp` | `ConstructedIsValidAndDelegates` | `avm-core/tests/camera.rs::constructed_camera_is_valid_and_delegates_dimensions` | ported |
| `test_camera.cpp` | `MovePreservesValidity` | `avm-core/tests/camera.rs::moved_camera_preserves_validity` | ported |

## `tests/camera_model/test_camera_extrinsics.cpp`

| C++ file | C++ case | Rust counterpart | Status |
| --- | --- | --- | --- |
| `test_camera_extrinsics.cpp` | `ToCamera_MatchesHandComputedRtPmT` | `avm-core/tests/camera_extrinsics.rs::to_camera_matches_hand_computed_rt_p_minus_t` | ported |
| `test_camera_extrinsics.cpp` | `ToWorldRoundTripsToCamera` | `avm-core/tests/camera_extrinsics.rs::to_world_round_trips_to_camera` | ported |
| `test_camera_extrinsics.cpp` | `ToWorld_MatchesHandComputedRPtT` | `avm-core/tests/camera_extrinsics.rs::to_world_matches_hand_computed_r_p_plus_t` | ported |
| `test_camera_extrinsics.cpp` | `RotateWorldToCamera_MatchesHandComputedRtV` | `avm-core/tests/camera_extrinsics.rs::rotate_world_to_camera_matches_hand_computed_rt_v` | ported |
| `test_camera_extrinsics.cpp` | `RotateCameraToWorld_MatchesHandComputedRv` | `avm-core/tests/camera_extrinsics.rs::rotate_camera_to_world_matches_hand_computed_r_v` | ported |
| `test_camera_extrinsics.cpp` | `Position_MatchesTVector` | `avm-core/tests/camera_extrinsics.rs::position_matches_translation_vector` | ported |
| `test_camera_extrinsics.cpp` | `DefaultConstructor_IsIdentity` | `avm-core/tests/camera_extrinsics.rs::default_constructor_is_identity` | ported |
| `test_camera_extrinsics.cpp` | `RelativeRotationFrom_MatchesHandComputedRbTRa` | `avm-core/tests/camera_extrinsics.rs::relative_rotation_from_matches_hand_computed_rb_t_ra` | ported |
| `test_camera_extrinsics.cpp` | `AzimuthDeg_IdentityAxisIsZero` | `avm-core/tests/camera_extrinsics.rs::azimuth_deg_of_identity_axis_is_zero` | ported |
| `test_camera_extrinsics.cpp` | `AzimuthDeg_ForwardAlongWorldYIs90` | `avm-core/tests/camera_extrinsics.rs::azimuth_deg_forward_along_world_y_is_90` | ported |
| `test_camera_extrinsics.cpp` | `AzimuthDeg_ForwardAlongWorld45Deg` | `avm-core/tests/camera_extrinsics.rs::azimuth_deg_forward_along_world_45_deg` | ported |
| `test_camera_extrinsics.cpp` | `ToCamera_OfOtherPosition_MatchesHandComputedTAb` | `avm-core/tests/camera_extrinsics.rs::to_camera_of_other_position_matches_hand_computed_ta_minus_tb` | ported |

## `tests/camera_model/test_camera_model.cpp`

| C++ file | C++ case | Rust counterpart | Status |
| --- | --- | --- | --- |
| `test_camera_model.cpp` | `Kb_ReturnsModel` | `avm-core/tests/camera_model_factory.rs::kb_returns_model_with_image_dimensions` | ported |
| `test_camera_model.cpp` | `UnknownModel_Throws` | `avm-core/tests/camera_model_factory.rs::unknown_model_is_rejected_and_named` | ported |
| `test_camera_model.cpp` | `VariantMismatch_Throws` | `avm-core/tests/camera_model_factory.rs::variant_mismatch_is_rejected_and_names_requested_model` | ported |
| `test_camera_model.cpp` | `NonPositiveSize_Throws` | `avm-core/tests/camera_model_factory.rs::non_positive_size_is_rejected_and_mentions_width` | ported |

## `tests/camera_model/test_kb_camera_model.cpp`

| C++ file | C++ case | Rust counterpart | Status |
| --- | --- | --- | --- |
| `test_kb_camera_model.cpp` | `IdentityCamera_Center` | `avm-core/tests/kb_camera_model.rs::identity_camera_projects_axis_point_to_principal_point` | ported |
| `test_kb_camera_model.cpp` | `IdentityCamera_OffCenter` | `avm-core/tests/kb_camera_model.rs::identity_camera_projects_off_center_point_with_equidistant_radius` | ported |
| `test_kb_camera_model.cpp` | `WithRotation` | `avm-core/tests/kb_camera_model.rs::rotated_camera_projects_axis_point_off_center` | ported |
| `test_kb_camera_model.cpp` | `WithTranslation` | `avm-core/tests/kb_camera_model.rs::translated_camera_projects_point_at_camera_origin_to_center` | ported |
| `test_kb_camera_model.cpp` | `MatchesOpenCV` | `avm-core/tests/kb_camera_model.rs::matches_opencv_fisheye_reference` | ported |
| `test_kb_camera_model.cpp` | `PointAtCameraZ` | `avm-core/tests/kb_camera_model.rs::point_at_camera_z_axis_projects_to_center` | ported |
| `test_kb_camera_model.cpp` | `LargeDistortionK1` | `avm-core/tests/kb_camera_model.rs::large_distortion_k1_follows_the_theta_polynomial` | ported |
| `test_kb_camera_model.cpp` | `PointBehindCamera_ReturnsValidUV` | `avm-core/tests/kb_camera_model.rs::point_behind_camera_still_returns_finite_uv` | ported |
| `test_kb_camera_model.cpp` | `DegenerateBranch_OnAxis` | `avm-core/tests/kb_camera_model.rs::kb_snaps_degenerate_rxy_to_principal_point` | ported |
| `test_kb_camera_model.cpp` | `InsideFisheyeCircle_OnBoundary` | `avm-core/tests/kb_camera_model.rs::inside_fisheye_circle_uses_inclusive_auto_radius` | ported |
| `test_kb_camera_model.cpp` | `EdgeFeather_NearEdge_HardGate_Zero` | `avm-core/tests/kb_camera_model.rs::edge_feather_hard_gate_excludes_the_rim_band` | ported |
| `test_kb_camera_model.cpp` | `EdgeFeather_ZeroFeatherWidth` | `avm-core/tests/kb_camera_model.rs::edge_feather_non_positive_width_disables_the_gate` | ported |
| `test_kb_camera_model.cpp` | `OnAxis_MatchesLegacy` | `avm-core/tests/kb_camera_model.rs::kb_factory_batch_projects_axis_point_exactly` | ported |
| `test_kb_camera_model.cpp` | `OffAxisZeroK_MatchesLegacy` | `avm-core/tests/kb_camera_model.rs::kb_factory_batch_projects_off_axis_point_without_distortion` | ported |
| `test_kb_camera_model.cpp` | `WithDistortion_MatchesLegacy` | `avm-core/tests/kb_camera_model.rs::kb_batch_matches_single_point_projection_with_distortion` | ported |
| `test_kb_camera_model.cpp` | `RXYBelowEps_SnapsToPrincipalPoint` | `avm-core/tests/kb_camera_model.rs::kb_snaps_degenerate_rxy_to_principal_point` | ported |
| `test_kb_camera_model.cpp` | `OutsideImageRectangle_Invalid` | `avm-core/tests/kb_camera_model.rs::kb_point_outside_image_rectangle_is_invalid` | ported |
| `test_kb_camera_model.cpp` | `BatchMatchesSingleCalls` | `avm-core/tests/kb_camera_model.rs::kb_batch_matches_single_calls_and_rejects_length_mismatch` | ported |
| `test_kb_camera_model.cpp` | `KbExplicitRadius_ForwardedToFisheyeCircle` | `avm-core/tests/kb_camera_model.rs::kb_explicit_fisheye_radius_is_forwarded_to_the_gate` | ported |
| `test_kb_camera_model.cpp` | `KbZeroRadius_AutoInscribedCircle` | `avm-core/tests/kb_camera_model.rs::kb_zero_radius_uses_the_auto_inscribed_circle` | ported |
| `test_kb_camera_model.cpp` | `Model_MatchesHandFormulaBitExact` | `avm-core/tests/kb_camera_model.rs::gating_matches_hand_formula_bit_exact_for_model_and_camera` | ported |

## `tests/camera_model/test_lut_camera_model.cpp`

| C++ file | C++ case | Rust counterpart | Status |
| --- | --- | --- | --- |
| `test_lut_camera_model.cpp` | `AxisPoint_ProjectsToPrincipalPoint` | `avm-core/tests/lut_camera_model.rs::axis_point_projects_to_principal_point` | ported |
| `test_lut_camera_model.cpp` | `Theta30_MatchesRendererKernel` | `avm-core/tests/lut_camera_model.rs::theta_30_lookup_returns_the_table_height_in_pixels` | ported |
| `test_lut_camera_model.cpp` | `TableEnd_ClampsLikeRenderer` | `avm-core/tests/lut_camera_model.rs::table_end_clamps_like_the_renderer` | ported |
| `test_lut_camera_model.cpp` | `InterpolatesBetweenRows` | `avm-core/tests/lut_camera_model.rs::angles_between_rows_interpolate_linearly` | ported |
| `test_lut_camera_model.cpp` | `NonAscendingPoints_Throws` | `avm-core/tests/lut_camera_model.rs::non_ascending_table_is_rejected_and_mentions_ascending` | ported |
| `test_lut_camera_model.cpp` | `SinglePoint_Throws` | `avm-core/tests/lut_camera_model.rs::single_point_table_is_rejected` | ported |
| `test_lut_camera_model.cpp` | `NonPositivePixelSize_Throws` | `avm-core/tests/lut_camera_model.rs::non_positive_pixel_size_is_rejected` | ported |
| `test_lut_camera_model.cpp` | `NonPositiveSize_Throws` | `avm-core/tests/lut_camera_model.rs::non_positive_size_is_rejected_and_mentions_width` | ported |
| `test_lut_camera_model.cpp` | `BatchMatchesSingle` | `avm-core/tests/lut_camera_model.rs::batch_projection_matches_single_point_calls` | ported |
| `test_lut_camera_model.cpp` | `OutsideRectangle_Invalid` | `avm-core/tests/lut_camera_model.rs::point_outside_image_rectangle_is_invalid` | ported |

## `tests/camera_model/test_ocam_camera_model.cpp`

| C++ file | C++ case | Rust counterpart | Status |
| --- | --- | --- | --- |
| `test_ocam_camera_model.cpp` | `AxisPoint_ProjectsToPrincipalPoint` | `avm-core/tests/ocam_camera_model.rs::axis_point_projects_to_principal_point` | ported |
| `test_ocam_camera_model.cpp` | `ReferencePoint_MatchesRendererKernel` | `avm-core/tests/ocam_camera_model.rs::reference_point_matches_the_renderer_kernel` | ported |
| `test_ocam_camera_model.cpp` | `RoundTrip_ClosesUnder01Px` | `avm-core/tests/ocam_camera_model.rs::ocam_round_trip_closes_under_0_01_px` | ported |
| `test_ocam_camera_model.cpp` | `BehindCamera_Invalid` | `avm-core/tests/ocam_camera_model.rs::behind_camera_is_invalid` | ported |
| `test_ocam_camera_model.cpp` | `EmptyInvpol_Throws` | `avm-core/tests/ocam_camera_model.rs::empty_invpol_is_rejected` | ported |
| `test_ocam_camera_model.cpp` | `NonPositiveSize_Throws` | `avm-core/tests/ocam_camera_model.rs::non_positive_size_is_rejected_and_mentions_width` | ported |
| `test_ocam_camera_model.cpp` | `BatchMatchesSingle` | `avm-core/tests/ocam_camera_model.rs::batch_projection_matches_single_calls_and_rejects_length_mismatch` | ported |

## `tests/common/test_angle_utils.cpp`

| C++ file | C++ case | Rust counterpart | Status |
| --- | --- | --- | --- |
| `test_angle_utils.cpp` | `NormalizeAngleDeg_KeepsInRangeUnchanged` | `avm-core/tests/angle_utils.rs::normalize_keeps_in_range_values_unchanged` | ported |
| `test_angle_utils.cpp` | `NormalizeAngleDeg_WrapsExactMultipleOf360ToZero` | `avm-core/tests/angle_utils.rs::normalize_wraps_exact_multiple_of_360_to_zero` | ported |
| `test_angle_utils.cpp` | `NormalizeAngleDeg_NegativeAdds360` | `avm-core/tests/angle_utils.rs::normalize_adds_360_to_negative_angles` | ported |
| `test_angle_utils.cpp` | `NormalizeAngleDeg_NegativeExactMultipleOf360ToZero` | `avm-core/tests/angle_utils.rs::normalize_maps_negative_multiple_of_360_to_zero` | ported |
| `test_angle_utils.cpp` | `NormalizeAngleDeg_MultiTurnWrapsIntoRange` | `avm-core/tests/angle_utils.rs::normalize_wraps_multiple_turns_into_range` | ported |
| `test_angle_utils.cpp` | `NormalizeAngleDeg_NonIntegerDegreesPreserved` | `avm-core/tests/angle_utils.rs::normalize_preserves_non_integer_degrees` | ported |
| `test_angle_utils.cpp` | `Constants_HaveExpectedValues` | `avm-core/tests/angle_utils.rs::constants_have_cpp_values` | ported |
| `test_angle_utils.cpp` | `Constants_ConvertSymmetric` | `avm-core/tests/angle_utils.rs::constants_convert_symmetrically` | ported |

## `tests/core/test_stitch_engine.cpp`

| C++ file | C++ case | Rust counterpart | Status |
| --- | --- | --- | --- |
| `test_stitch_engine.cpp` | `ConfigureN0Throws` | `avm-engine/tests/engine.rs::configure_n0_is_rejected` | ported |
| `test_stitch_engine.cpp` | `ConfigureN9Throws` | `avm-engine/tests/engine.rs::configure_n9_is_rejected` | ported |
| `test_stitch_engine.cpp` | `ConfigureMismatchedImageSizesThrows` | `avm-engine/tests/engine.rs::configure_mismatched_image_sizes_is_rejected` | ported |
| `test_stitch_engine.cpp` | `ConfigureNullCameraModelThrows` | `avm-engine/tests/engine.rs::configure_null_camera_model_is_rejected` | ported |
| `test_stitch_engine.cpp` | `ConfigureN1_N8_Success` | `avm-engine/tests/engine.rs::configure_accepts_one_to_eight_cameras` | ported |
| `test_stitch_engine.cpp` | `IsConfiguredFalseBefore` | `avm-engine/tests/engine.rs::is_configured_is_false_before_configure` | ported |
| `test_stitch_engine.cpp` | `IsConfiguredTrueAfter` | `avm-engine/tests/engine.rs::is_configured_is_true_after_configure` | ported |
| `test_stitch_engine.cpp` | `GetNumCameras` | `avm-engine/tests/engine.rs::num_cameras_reports_the_configured_count` | ported |
| `test_stitch_engine.cpp` | `RenderFrameBeforeConfigureThrows` | `avm-engine/tests/engine.rs::render_frame_before_configure_is_rejected` | ported |
| `test_stitch_engine.cpp` | `RenderFrameWrongImageCount` | `avm-engine/tests/engine.rs::render_frame_with_the_wrong_image_count_is_rejected` | ported |
| `test_stitch_engine.cpp` | `Configure_BowlParamsCarLengthWidth` | `avm-engine/tests/engine.rs::configure_accepts_an_overridden_bowl_size` | ported |
| `test_stitch_engine.cpp` | `ConfigureStepZeroThrows` | `avm-engine/tests/engine.rs::configure_rejects_a_zero_grid_step` | ported |
| `test_stitch_engine.cpp` | `ConfigureNonPositiveOuterRadiusThrows` | `avm-engine/tests/engine.rs::configure_rejects_a_non_positive_outer_radius` | ported |
| `test_stitch_engine.cpp` | `ConfigureNegativeKSlopThrows` | `avm-engine/tests/engine.rs::configure_rejects_a_negative_slope_coefficient` | ported |
| `test_stitch_engine.cpp` | `ConfigureNegativeZExpThrows` | `avm-engine/tests/engine.rs::configure_rejects_a_negative_height_exponent` | ported |
| `test_stitch_engine.cpp` | `StitchEngineGL_RenderFrame_Returns4Views` | `avm-engine/tests/engine.rs::render_frame_returns_four_named_views` | ported |
| `test_stitch_engine.cpp` | `ConfigureKbPath_Configures_AndRenders4Views` | `avm-engine/tests/engine.rs::configure_kb_path_renders_four_views` | ported |
| `test_stitch_engine.cpp` | `StitchEngineGL_RenderFrame_OutputSize` | `avm-engine/tests/engine.rs::render_frame_output_size_matches_the_engine_default` | ported |
| `test_stitch_engine.cpp` | `StitchEngineGL_RenderFrame_NonBlackOutput` | `avm-engine/tests/engine.rs::render_frame_output_is_not_black` | ported |
| `test_stitch_engine.cpp` | `StitchEngineGL_RenderFrame_N8Cameras` | `avm-engine/tests/engine.rs::render_frame_with_eight_cameras` | ported |
| `test_stitch_engine.cpp` | `StitchEngineGL_FrontViewNotVerticallyFlipped` | `avm-engine/tests/engine.rs::front_view_is_not_vertically_flipped` | ported |
| `test_stitch_engine.cpp` | `PublicApiHeadersAreSelfContained` | `avm-engine/tests/engine.rs::public_api_headers_are_self_contained` | ported |

## `tests/depth/test_ground_depth.cpp`

| C++ file | C++ case | Rust counterpart | Status |
| --- | --- | --- | --- |
| `test_ground_depth.cpp` | `GroundDepthKnownCase` | `avm-core/tests/depth.rs::ground_depth_known_case` | ported |
| `test_ground_depth.cpp` | `DisparityMonotonicWithDepth` | `avm-core/tests/depth.rs::disparity_monotonic_with_depth` | ported |
| `test_ground_depth.cpp` | `TriangulateDepthRoundTrip` | `avm-core/tests/depth.rs::triangulate_depth_round_trip` | ported |
| `test_ground_depth.cpp` | `InvalidOutsideRange` | `avm-core/tests/depth.rs::invalid_outside_range` | ported |
| `test_ground_depth.cpp` | `MapShapes` | `avm-core/tests/depth.rs::map_shapes` | ported |
| `test_ground_depth.cpp` | `DisparityAnalyticValue` | `avm-core/tests/depth.rs::disparity_analytic_value` | ported |
| `test_ground_depth.cpp` | `DepthRangeBoundsInvalid` | `avm-core/tests/depth.rs::depth_range_bounds_invalid` | ported |

## `tests/depth/test_spherical_rectification.cpp`

| C++ file | C++ case | Rust counterpart | Status |
| --- | --- | --- | --- |
| `test_spherical_rectification.cpp` | `DetIsRightHanded` | `avm-core/tests/depth.rs::det_is_right_handed` | ported |
| `test_spherical_rectification.cpp` | `BasisIsOrthonormal` | `avm-core/tests/depth.rs::basis_is_orthonormal` | ported |
| `test_spherical_rectification.cpp` | `BaselinePointsFromAToB` | `avm-core/tests/depth.rs::baseline_points_from_a_to_b` | ported |
| `test_spherical_rectification.cpp` | `RowAlignmentHoldsForRandomPoints` | `avm-core/tests/depth.rs::row_alignment_holds_for_random_points` | ported |
| `test_spherical_rectification.cpp` | `GridDirectionIsUnit` | `avm-core/tests/depth.rs::grid_direction_is_unit` | ported |
| `test_spherical_rectification.cpp` | `MirrorKeepsBasisRightHanded` | `avm-core/tests/depth.rs::mirror_keeps_basis_right_handed` | ported |
| `test_spherical_rectification.cpp` | `SkyUpNeedsMirrorDetectsFlippedRowOrder` | `avm-core/tests/depth.rs::sky_up_needs_mirror_detects_flipped_row_order` | ported |

## `tests/render/test_osg_view_renderer.cpp`

| C++ file | C++ case | Rust counterpart | Status |
| --- | --- | --- | --- |
| `test_osg_view_renderer.cpp` | `DefaultViewPoses_Returns4Views` | `avm-core/tests/view_presets.rs::default_view_poses_returns_4_views` | ported |
| `test_osg_view_renderer.cpp` | `DefaultViewPoses_HasCorrectNames` | `avm-core/tests/view_presets.rs::default_view_poses_have_correct_names` | ported |
| `test_osg_view_renderer.cpp` | `DefaultViewPoses_AllFovy60` | `avm-core/tests/view_presets.rs::default_view_poses_all_use_fovy_60` | ported |
| `test_osg_view_renderer.cpp` | `DefaultViewPoses_AllTargetSame` | `avm-core/tests/view_presets.rs::default_view_poses_share_target_and_up` | ported |
| `test_osg_view_renderer.cpp` | `DefaultViewPoses_CarCenterOffsetsAllPoses` | `avm-core/tests/view_presets.rs::default_view_poses_are_offset_by_the_car_center` | ported |
| `test_osg_view_renderer.cpp` | `DefaultViewPoses_ScaleOneMatchesHistoricalConstants` | `avm-core/tests/view_presets.rs::default_view_poses_at_scale_one_match_historical_constants` | ported |
| `test_osg_view_renderer.cpp` | `DefaultViewPoses_ScaleMultipliesOffsetsAndHeights` | `avm-core/tests/view_presets.rs::default_view_poses_scale_offsets_and_heights` | ported |
| `test_osg_view_renderer.cpp` | `HomeViewPose_BowlParamsAgreesWithHandBuiltScale` | `avm-core/tests/view_presets.rs::home_view_pose_agrees_with_the_hand_built_scale` | ported |
| `test_osg_view_renderer.cpp` | `HomeViewPose_CarCenterOffsetsPositionAndTarget` | `avm-core/tests/view_presets.rs::home_view_pose_car_center_offsets_position_and_target` | ported |
| `test_osg_view_renderer.cpp` | `DefaultViewPoses_BowlParamsAgreesWithHandBuiltScale` | `avm-core/tests/view_presets.rs::default_view_poses_agree_with_the_hand_built_scale_for_car_sizes` | ported |
| `test_osg_view_renderer.cpp` | `ClipPlanesFor_CarScale_CoversBowl` | `avm-core/tests/view_presets.rs::clip_planes_car_scale_cover_the_bowl` | ported |
| `test_osg_view_renderer.cpp` | `ClipPlanesFor_ShipScale_CoversBowl` | `avm-core/tests/view_presets.rs::clip_planes_ship_scale_cover_the_bowl` | ported |
| `test_osg_view_renderer.cpp` | `ClipPlanesFor_ScaleInvariant` | `avm-core/tests/view_presets.rs::clip_planes_are_scale_invariant` | ported |
| `test_osg_view_renderer.cpp` | `OSGViewRenderer_ConstructionDoesNotCrash` | ① EGL/OSG 实现专属：构造 `osgViewer::Viewer` 与 `osg::Camera` 没有 wgpu 对应概念 | exempt |
| `test_osg_view_renderer.cpp` | `OSGViewRenderer_RenderAllViews_ShipScale_NotAllBackground` | `avm-render/tests/offscreen.rs::ship_scale_views_are_not_background` | ported |
| `test_osg_view_renderer.cpp` | `OSGViewRenderer_OffscreenMode_Works` | `avm-render/tests/offscreen.rs::renders_every_view_or_skips_without_adapter` | ported |
| `test_osg_view_renderer.cpp` | `OSGViewRenderer_WindowedMode_Constructs` | ④ 需要可交互显示：`EventLoop`/窗口构造在无显示的 CI 中无法自动化；`--windowed` 入口与 `avm-viewer` 控制器测试覆盖了窗口所用的相机与交互数学 | exempt |
| `test_osg_view_renderer.cpp` | `OSGViewRenderer_WindowedMode_Constructs_WithCarCenter` | ④ 同上；车心对初始相机的影响由 `avm-viewer/tests/controller.rs::home_camera_matches_the_home_view_pose` 覆盖（fixture 的 car_center 非零） | exempt |

## `tests/render/test_stitch_scene.cpp`

| C++ file | C++ case | Rust counterpart | Status |
| --- | --- | --- | --- |
| `test_stitch_scene.cpp` | `BowlGeometryPositionAttributeCorrect` | `avm-render/tests/vertex_packing.rs::position_and_normal_use_the_osg_render_basis` | ported |
| `test_stitch_scene.cpp` | `BowlGeometryUVArraysExist` | `avm-render/tests/vertex_packing.rs::from_baked_packs_two_cameras_per_uv_slot_and_four_per_weight_slot` | ported |
| `test_stitch_scene.cpp` | `BowlGeometryWeightArraysExist` | `avm-render/tests/vertex_packing.rs::from_baked_packs_two_cameras_per_uv_slot_and_four_per_weight_slot` | ported |
| `test_stitch_scene.cpp` | `BowlGeometryIndexCount` | `avm-render/tests/vertex_packing.rs::scene_data_packs_the_bowl_mesh_and_index_buffer` | ported |
| `test_stitch_scene.cpp` | `BowlGeometryNormalBinding` | `avm-render/tests/vertex_packing.rs::position_and_normal_use_the_osg_render_basis` | ported |
| `test_stitch_scene.cpp` | `EmptySlotsZeroPaddedWhenBakingFourCameras` | `avm-render/tests/vertex_packing.rs::from_baked_zero_fills_cameras_beyond_eight` | ported |
| `test_stitch_scene.cpp` | `EmptySlotsZeroPaddedWhenBakingTwoCameras` | `avm-render/tests/vertex_packing.rs::from_baked_zero_fills_cameras_beyond_eight` | ported |
| `test_stitch_scene.cpp` | `ShaderSources_VertexCompilesSyntactically` | `avm-render/tests/shader_validation.rs::shader_parses_and_validates_with_naga` | ported |
| `test_stitch_scene.cpp` | `ShaderSources_FragmentCompilesSyntactically` | `avm-render/tests/shader_validation.rs::shader_parses_and_validates_with_naga` | ported |
| `test_stitch_scene.cpp` | `ShaderSources_All8CamerasUnrolled` | ① GLSL 展开 8 路采样是 OSG 着色器的实现细节；Rust 着色器用循环 + `MAX_CAMERAS`，由 `shader_uses_a_texture_array_and_the_hard_weight_gate` 覆盖 | exempt |
| `test_stitch_scene.cpp` | `CameraTextureArraySetsCorrectSize` | ③ `osg::Texture2DArray::setImage` 的内部尺寸断言，wgpu 侧由纹理创建参数与渲染路径覆盖 | exempt |
| `test_stitch_scene.cpp` | `NumCamerasComesFromBakedBowl` | `avm-render/tests/vertex_packing.rs::scene_data_packs_the_bowl_mesh_and_index_buffer` | ported |
| `test_stitch_scene.cpp` | `RootNonNullGroup` | ③ `osg::Group` 根节点结构断言；wgpu 渲染路径没有场景图 | exempt |
| `test_stitch_scene.cpp` | `ProgramAttached` | ③ `osg::StateSet` 程序绑定断言；wgpu 用固定管线对象 | exempt |
| `test_stitch_scene.cpp` | `UploadCameraImages_DoesNotCrash` | `avm-render/tests/offscreen.rs::every_camera_slot_samples_its_own_layer_and_uv_slot` | ported |

## `tests/render/test_egl_headless_context.cpp`

| C++ file | C++ case | Rust counterpart | Status |
| --- | --- | --- | --- |
| `test_egl_headless_context.cpp` | `RealizesValidHeadless` | ① EGL/OSG 实现专属：headless EGL 上下文在 wgpu 中没有对应概念 | exempt |
| `test_egl_headless_context.cpp` | `MakeCurrentExposesDesktopGL` | ① EGL/OSG 实现专属：headless EGL 上下文在 wgpu 中没有对应概念 | exempt |
| `test_egl_headless_context.cpp` | `GLSL330Compiles` | ① EGL/OSG 实现专属：headless EGL 上下文在 wgpu 中没有对应概念 | exempt |
| `test_egl_headless_context.cpp` | `ReleaseThenRemakeCurrent` | ① EGL/OSG 实现专属：headless EGL 上下文在 wgpu 中没有对应概念 | exempt |

