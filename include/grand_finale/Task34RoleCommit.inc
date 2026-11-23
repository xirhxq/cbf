    // Explicit role-request seam. No existing/default path calls it.
    const std::optional<std::map<NodeId,Eigen::Vector2d>>& task34ReconstructionReference() const {
        return external_reconstruction_reference_;
    }
    void registerExternalRoleLibrary(const Task34RegisteredRoles& registry) {
        if(adapter_.runtimeSnapshot().runtime_s!=0||target_epoch_!=0)
            throw std::logic_error("role_library_must_precede_motion");
        const auto key=[](const Task20DagLatticeContract& c){return Task34FrontMotionHistory(c).snapshot().at("contract");};
        for(const int code:{0,11,12,1,2,13}) {
            std::map<NodeId,NodeId> identity;
            for(const auto id:adapter_.runtimeSnapshot().estimate.mobile_ids)identity[id]=id;
            const auto mode=registry.instantiate(code,identity);
            if(mode.fixed!=adapter_.runtimeSnapshot().estimate.fixed_positions)
                throw std::logic_error("role_library_physical_scene_mismatch");
            const auto found=registered_external_contracts_.find(code);
            if(found==registered_external_contracts_.end())registerExternalCoverageContract(code,mode.contract);
            else if(key(found->second)!=key(mode.contract))throw std::logic_error("role_library_conflicts_with_startup_template");
        }
    }
    void commitExternalRoleCoverageMode(const Task34RegisteredRoles& registry,
        const nlohmann::json& frozen_instance) {
        // Reconstruct, rather than accepting a mutable caller-supplied contract.
        const auto instance=registry.restore(frozen_instance);
        std::map<NodeId,NodeId> identity;
        for(const auto& [id,r]:instance.contract.member_roles)identity[id]=id;
        const auto original=registry.instantiate(instance.mode_code,identity);
        const auto registered=registered_external_contracts_.find(instance.mode_code);
        const auto key=[](const Task20DagLatticeContract& c){return Task34FrontMotionHistory(c).snapshot().at("contract");};
        if(registered==registered_external_contracts_.end()||key(registered->second)!=key(original.contract))
            throw std::logic_error("role_instance_not_from_startup_template");
        const auto runtime=adapter_.runtimeSnapshot();
        if(!adapter_.config().target_policy_task20_dag_lattice||
            runtime.estimate.fixed_positions!=instance.fixed||
            runtime.mode!=SupervisorMode::Search||runtime.adapter_transition_pending||
            task25_detail::edgeSet(runtime.topology)!=task25_detail::edgeSet(instance.contract.reference_edges))
            throw std::logic_error("role_mode_cannot_precede_actual_full_dag_handoff");
        // This commits no edges or EKF state. Fresh make/break qualification
        // remains the adapter's responsibility before this point.
        external_coverage_mode_=instance.mode_code;
        external_coverage_contract_=instance.contract;
        external_force_reallocate_=true;
    }
