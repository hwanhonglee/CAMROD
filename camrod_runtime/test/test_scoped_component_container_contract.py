from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_container_owns_and_finalizes_explicit_context():
    source = (ROOT / "src" / "scoped_component_container.cpp").read_text(
        encoding="utf-8"
    )

    assert ".context(context)" in source
    assert ".context(std::move(context))" in source
    assert "rclcpp::uninstall_signal_handlers()" in source
    assert "std::signal(SIGINT, SIG_IGN)" in source
    assert "std::signal(SIGTERM, SIG_IGN)" in source
    assert source.index("rclcpp::uninstall_signal_handlers()") < source.index(
        "manager->detach_components_and_release_loaders()"
    )
    assert source.index(
        "manager->detach_components_and_release_loaders()"
    ) < source.index("manager.reset()")
    context_reset = source.index("\n  context.reset();")
    assert source.index("manager.reset()") < context_reset
    assert source.index("executor_options.context.reset()") < source.index(
        "\n  context.reset();"
    )


def test_components_leave_executor_before_plugin_unload():
    source = (ROOT / "src" / "scoped_component_container.cpp").read_text(
        encoding="utf-8"
    )

    assert "remove_node_from_executor(entry.first)" in source
    assert "return std::move(loaders_)" in source
    assert source.index("node_wrappers_.clear()") < source.index(
        "return std::move(loaders_)"
    )
    assert source.index("\n  context.reset();") < source.index("plugin_loaders.clear()")


def test_both_executor_variants_are_installed():
    cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    assert "camrod_add_scoped_container(scoped_component_container)" in cmake
    assert "camrod_add_scoped_container(scoped_component_container_mt)" in cmake


def test_process_exit_skips_humble_dso_destructor_race_after_cleanup():
    source = (ROOT / "src" / "scoped_component_container.cpp").read_text(
        encoding="utf-8"
    )

    assert "std::_Exit(exit_code)" in source
    assert source.index("plugin_loaders.clear()") < source.index(
        "std::_Exit(exit_code)"
    )
