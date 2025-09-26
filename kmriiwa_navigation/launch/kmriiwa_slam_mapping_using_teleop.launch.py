# kmriiwa_slam_config.yaml - ROS1 Bridge Compatible
slam_toolbox:
  ros__parameters:
    # Use real time
    use_sim_time: false
    
    # Frame names - check what your robot actually publishes
    odom_frame: odom  # Changed from kmriiwa_odom - verify this matches your actual frame
    map_frame: map
    base_frame: base_link
    
    # Topics
    scan_topic: /scan_multi
    odom_topic: /kmriiwa/base/state/odom
    
    # Timing and buffer parameters - increase to handle queue full errors
    minimum_time_interval: 1.0  # Increased from 0.5
    transform_timeout: 3.0  # Increased from 1.0
    tf_buffer_duration: 30.0  # Increased from 15.0
    
    # Scan processing - reduce load
    scan_buffer_size: 25  # Increased from 10
    scan_buffer_maximum_scan_distance: 15.0  # Reduced from 20.0
    minimum_travel_distance: 0.3  # Increased from 0.2
    map_update_interval: 8.0  # Increased from 5.0
    
    # Motion parameters
    linear_update_distance: 0.1
    angular_update_distance: 0.1
    
    # Loop closure
    do_loop_closing: true
    loop_search_maximum_distance: 3.0
    loop_search_space_dimension: 8.0
    
    # Map saving
    map_start_pose: [0.0, 0.0, 0.0]
    
    # Scan matching
    use_scan_matching: true
    use_scan_barycenter: true
    minimum_travel_distance: 0.2
    minimum_travel_heading: 0.1
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 20.0
    
    # Performance parameters
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45
    
    # Correlation search
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1
    
    # Map update
    map_update_interval: 5.0
    
    # Interactive mode
    enable_interactive_mode: true
    
    # QoS settings
    qos_overrides:
      /scan_multi:
        reliability: best_effort
        durability: volatile
        history: keep_last
        depth: 10