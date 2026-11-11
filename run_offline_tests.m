function run_offline_tests
% Synthetic function tests only. No model Start, target connection or build.
setup_project;
tests={@test_flow_assist,@test_sb_mode,@test_roll_yaw_controller, ...
    @test_gyrocal,@test_roll_i,@test_yaw_i,@test_qdrone_capture, ...
    @test_baro_height,@test_baro_height_kalman,@test_height_lp_hp};
for k=1:numel(tests)
    fprintf('\n=== %s ===\n',func2str(tests{k}));
    tests{k}();
end
fprintf('\nPASS: %d offline test groups. Hardware flight has not been validated.\n',numel(tests));
end
