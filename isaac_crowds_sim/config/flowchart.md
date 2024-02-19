```mermaid
flowchart TD;
    master[Airport] --> size_space{Size of Space}

    size_space --> space_small(Small space)
    size_space --> space_big(Big space)

    space_small --> crowd_density_small{Crowd Density}
    space_big --> crowd_density_big{Crowd Density}

    crowd_density_small --> no_crowd_small_space(No crowd)
    crowd_density_small --> small_crowd_small_space(Small crowd)
    crowd_density_small --> big_crowd_small_space(Big crowd)

    crowd_density_big --> no_crowd_big_space(No crowd)
    crowd_density_big --> small_crowd_big_space(Small crowd)
    crowd_density_big --> big_crowd_big_space(Big crowd)

    small_crowd_small_space --> crowd_direction_small_crowd_small_space{Crowd Direction}
    big_crowd_small_space --> crowd_direction_big_crowd_small_space{Crowd Direction}
    small_crowd_big_space --> crowd_direction_small_crowd_big_space{Crowd Direction}
    big_crowd_big_space --> crowd_direction_big_crowd_big_space{Crowd Direction}

    crowd_direction_small_crowd_small_space --> static_small_crowd_small_space(Static Crowd)
    crowd_direction_big_crowd_small_space --> static_big_crowd_small_space(Static Crowd)
    crowd_direction_small_crowd_big_space --> static_small_crowd_big_space(Static Crowd)
    crowd_direction_big_crowd_big_space --> static_big_crowd_big_space(Static Crowd)

    crowd_direction_small_crowd_small_space --> directional_small_crowd_small_space(Directional Crowd)
    crowd_direction_big_crowd_small_space --> directional_big_crowd_small_space(Directional Crowd)
    crowd_direction_small_crowd_big_space --> directional_small_crowd_big_space(Directional Crowd)
    crowd_direction_big_crowd_big_space --> directional_big_crowd_big_space(Directional Crowd)

    crowd_direction_small_crowd_small_space --> unordered_small_crowd_small_space(Unordered Dynamic Crowd)
    crowd_direction_big_crowd_small_space --> unordered_big_crowd_small_space(Unordered Dynamic Crowd)
    crowd_direction_small_crowd_big_space --> unordered_small_crowd_big_space(Unordered Dynamic Crowd)
    crowd_direction_big_crowd_big_space --> unordered_big_crowd_big_space(Unordered Dynamic Crowd)

    static_small_crowd_small_space --> obstacles_static_small_crowd_small_space{Static Obstacles}
    static_big_crowd_small_space --> obstacles_static_big_crowd_small_space{Static Obstacles}
    static_small_crowd_big_space --> obstacles_static_small_crowd_big_space{Static Obstacles}
    static_big_crowd_big_space --> obstacles_static_big_crowd_big_space{Static Obstacles}

    directional_small_crowd_small_space --> obstacles_directional_small_crowd_small_space{Static Obstacles}
    directional_big_crowd_small_space --> obstacles_directional_big_crowd_small_space{Static Obstacles}
    directional_small_crowd_big_space --> obstacles_directional_small_crowd_big_space{Static Obstacles}
    directional_big_crowd_big_space --> obstacles_directional_big_crowd_big_space{Static Obstacles}

    unordered_small_crowd_small_space --> obstacles_unordered_small_crowd_small_space{Static Obstacles}
    unordered_big_crowd_small_space --> obstacles_unordered_big_crowd_small_space{Static Obstacles}
    unordered_small_crowd_big_space --> obstacles_unordered_small_crowd_big_space{Static Obstacles}
    unordered_big_crowd_big_space --> obstacles_unordered_big_crowd_big_space{Static Obstacles}

    no_crowd_small_space ----> obstacles_no_crowd_small_space{Static Obstacles}
    no_crowd_big_space ----> obstacles_no_crowd_big_space{Static Obstacles}

    obstacles_no_crowd_small_space --> no_obstacles_no_crowd_small_space(No Obstacles)
    obstacles_no_crowd_small_space --> few_obstacles_no_crowd_small_space(Few Obstacles)
    obstacles_no_crowd_small_space --> many_obstacles_no_crowd_small_space(Many Obstacles)

    obstacles_no_crowd_big_space --> no_obstacles_no_crowd_big_space(No Obstacles)
    obstacles_no_crowd_big_space --> few_obstacles_no_crowd_big_space(Few Obstacles)
    obstacles_no_crowd_big_space --> many_obstacles_no_crowd_big_space(Many Obstacles)

    obstacles_static_small_crowd_small_space --> no_obstacles_static_small_crowd_small_space(No Obstacles)
    obstacles_static_small_crowd_small_space --> few_obstacles_static_small_crowd_small_space(Few Obstacles)
    obstacles_static_small_crowd_small_space --> many_obstacles_static_small_crowd_small_space(Many Obstacles)

    obstacles_static_small_crowd_big_space --> no_obstacles_static_small_crowd_big_space(No Obstacles)
    obstacles_static_small_crowd_big_space --> few_obstacles_static_small_crowd_big_space(Few Obstacles)
    obstacles_static_small_crowd_big_space --> many_obstacles_static_small_crowd_big_space(Many Obstacles)

    obstacles_static_big_crowd_small_space --> no_obstacles_static_big_crowd_small_space(No Obstacles)
    obstacles_static_big_crowd_small_space --> few_obstacles_static_big_crowd_small_space(Few Obstacles)
    obstacles_static_big_crowd_small_space --> many_obstacles_static_big_crowd_small_space(Many Obstacles)

    obstacles_static_big_crowd_big_space --> no_obstacles_static_big_crowd_big_space(No Obstacles)
    obstacles_static_big_crowd_big_space --> few_obstacles_static_big_crowd_big_space(Few Obstacles)
    obstacles_static_big_crowd_big_space --> many_obstacles_static_big_crowd_big_space(Many Obstacles)

    obstacles_directional_small_crowd_small_space --> no_obstacles_directional_small_crowd_small_space(No Obstacles)
    obstacles_directional_small_crowd_small_space --> few_obstacles_directional_small_crowd_small_space(Few Obstacles)
    obstacles_directional_small_crowd_small_space --> many_obstacles_directional_small_crowd_small_space(Many Obstacles)

    obstacles_directional_small_crowd_big_space --> no_obstacles_directional_small_crowd_big_space(No Obstacles)
    obstacles_directional_small_crowd_big_space --> few_obstacles_directional_small_crowd_big_space(Few Obstacles)
    obstacles_directional_small_crowd_big_space --> many_obstacles_directional_small_crowd_big_space(Many Obstacles)

    obstacles_directional_big_crowd_small_space --> no_obstacles_directional_big_crowd_small_space(No Obstacles)
    obstacles_directional_big_crowd_small_space --> few_obstacles_directional_big_crowd_small_space(Few Obstacles)
    obstacles_directional_big_crowd_small_space --> many_obstacles_directional_big_crowd_small_space(Many Obstacles)

    obstacles_directional_big_crowd_big_space --> no_obstacles_directional_big_crowd_big_space(No Obstacles)
    obstacles_directional_big_crowd_big_space --> few_obstacles_directional_big_crowd_big_space(Few Obstacles)
    obstacles_directional_big_crowd_big_space --> many_obstacles_directional_big_crowd_big_space(Many Obstacles)

    obstacles_unordered_small_crowd_small_space --> no_obstacles_unordered_small_crowd_small_space(No Obstacles)
    obstacles_unordered_small_crowd_small_space --> few_obstacles_unordered_small_crowd_small_space(Few Obstacles)
    obstacles_unordered_small_crowd_small_space --> many_obstacles_unordered_small_crowd_small_space(Many Obstacles)

    obstacles_unordered_small_crowd_big_space --> no_obstacles_unordered_small_crowd_big_space(No Obstacles)
    obstacles_unordered_small_crowd_big_space --> few_obstacles_unordered_small_crowd_big_space(Few Obstacles)
    obstacles_unordered_small_crowd_big_space --> many_obstacles_unordered_small_crowd_big_space(Many Obstacles)

    obstacles_unordered_big_crowd_small_space --> no_obstacles_unordered_big_crowd_small_space(No Obstacles)
    obstacles_unordered_big_crowd_small_space --> few_obstacles_unordered_big_crowd_small_space(Few Obstacles)
    obstacles_unordered_big_crowd_small_space --> many_obstacles_unordered_big_crowd_small_space(Many Obstacles)

    obstacles_unordered_big_crowd_big_space --> no_obstacles_unordered_big_crowd_big_space(No Obstacles)
    obstacles_unordered_big_crowd_big_space --> few_obstacles_unordered_big_crowd_big_space(Few Obstacles)
    obstacles_unordered_big_crowd_big_space --> many_obstacles_unordered_big_crowd_big_space(Many Obstacles)

    no_obstacles_no_crowd_small_space --> toilet[Toilet]
    few_obstacles_no_crowd_small_space --> toilet[Toilet]
    many_obstacles_no_crowd_small_space --> empty_shop[Empty Shop]

    no_obstacles_static_small_crowd_small_space --> toilet[Toilet]
    few_obstacles_static_small_crowd_small_space --> toilet[Toilet]
    many_obstacles_static_small_crowd_small_space --> shop[Shop]

    no_obstacles_directional_small_crowd_small_space --> toilet[Toilet]
    few_obstacles_directional_small_crowd_small_space --> toilet[Toilet]
    many_obstacles_directional_small_crowd_small_space --> shop[Shop]

    no_obstacles_unordered_small_crowd_small_space --> toilet[Toilet]
    few_obstacles_unordered_small_crowd_small_space --> toilet[Toilet]
    many_obstacles_unordered_small_crowd_small_space --> shop[Shop]

    no_obstacles_static_big_crowd_small_space --> toilet[Toilet]
    few_obstacles_static_big_crowd_small_space --> toilet[Toilet]
    many_obstacles_static_big_crowd_small_space --> shop[Shop]

    no_obstacles_directional_big_crowd_small_space --> toilet[Toilet]
    few_obstacles_directional_big_crowd_small_space --> toilet[Toilet]
    many_obstacles_directional_big_crowd_small_space --> shop[Shop]

    no_obstacles_unordered_big_crowd_small_space --> toilet[Toilet]
    few_obstacles_unordered_big_crowd_small_space --> toilet[Toilet]
    many_obstacles_unordered_big_crowd_small_space --> shop[Shop]

    no_obstacles_no_crowd_big_space --> open_floor[Open Floor]
    few_obstacles_no_crowd_big_space --> open_floor[Open Floor]
    many_obstacles_no_crowd_big_space --> empty_shop[Empty Shop]

    no_obstacles_static_small_crowd_big_space --> queue[Queue]
    few_obstacles_static_small_crowd_big_space --> queue[Queue]
    many_obstacles_static_small_crowd_big_space --> shop[Shop]

    no_obstacles_directional_small_crowd_big_space --> passage[Passage]
    few_obstacles_directional_small_crowd_big_space --> passage[Passage]
    many_obstacles_directional_small_crowd_big_space --> shop[Shop]

    no_obstacles_unordered_small_crowd_big_space --> floor[Floor]
    few_obstacles_unordered_small_crowd_big_space --> floor[Floor]
    many_obstacles_unordered_small_crowd_big_space --> shop[Shop]

    no_obstacles_static_big_crowd_big_space --> queue[Queue]
    few_obstacles_static_big_crowd_big_space --> queue[Queue]
    many_obstacles_static_big_crowd_big_space --> full_shop[Full Shop]

    no_obstacles_directional_big_crowd_big_space --> full_passage[Full Passage]
    few_obstacles_directional_big_crowd_big_space --> full_passage[Full Passage]
    many_obstacles_directional_big_crowd_big_space --> full_shop[Full Shop]

    no_obstacles_unordered_big_crowd_big_space --> full_floor[Full Floor]
    few_obstacles_unordered_big_crowd_big_space --> full_floor[Full Floor]
    many_obstacles_unordered_big_crowd_big_space --> full_shop[Full Shop]

    toilet --> manual_control[Manual Control]
    full_shop --> manual_control[Manual Control]

    queue --> shared_control[Shared Control]
    shop --> shared_control[Shared Control]
    full_passage --> shared_control[Shared Control]
    floor --> shared_control[Shared Control]
    full_floor --> shared_control[Shared Control]

    empty_shop --> full_control[Full Control]
    open_floor --> full_control[Full Control]
```