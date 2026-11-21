"""Create isolated source candidate; does not change authoritative deployment."""
from pathlib import Path
import difflib,json,hashlib
out=Path(__file__).resolve().parent/'candidate'; out.mkdir(exist_ok=True)
src=Path(r'X:/Items/as2w/AS2-W_CPRO/部署参考/cpp_deploy-as2w/deploy/robots/as2w')
provenance={}
for rel in ['himloco/himloco_controller.hpp','himloco/himloco_controller.cpp','common/as2w_publish.cpp']:
    original=(src/rel).read_text(encoding='utf-8'); changed=original
    if rel.endswith('.hpp'):
        changed=changed.replace('    void stop();','    void stop();\n    void accept_applied(const Targets& applied);')
        changed=changed.replace('    Motors target_leg_{};', '    Motors applied_leg_{};\n    std::array<float, 4> applied_wheel_{};\n    Motors target_leg_{};')
    elif rel.endswith('himloco_controller.cpp'):
        changed=changed.replace('    target_leg_.fill(0.f);','    applied_leg_.fill(0.f);\n    applied_wheel_.fill(0.f);\n    target_leg_.fill(0.f);')
        marker='Targets Controller::step('
        method='''void Controller::accept_applied(const Targets& applied) {
    // Separate observation feedback from the internal low-pass filter state.
    // Called after the publisher's blend/filter, in policy joint coordinates.
    for (std::size_t i = 0; i < LegAxes; ++i)
        applied_leg_[i] = applied.motors[i].q - config_.defaults[i];
    for (std::size_t i = 0; i < 4; ++i)
        applied_wheel_[i] = applied.motors[12 + i].dq;
}

'''
        changed=changed.replace(marker,method+marker)
        changed=changed.replace('frame[k++] = target_leg_[i] /','frame[k++] = applied_leg_[i] /')
        changed=changed.replace('frame[k++] = target_wheel_[i] /','frame[k++] = applied_wheel_[i] /')
        # Direct replay callers have no publisher blend: preserve that behavior.
        at=changed.rfind('    return out;\n}')
        assert at>changed.index('Targets Controller::step(')
        changed=changed[:at]+'    accept_applied(out);\n'+changed[at:]
    else:
        for phase,variable in [('HANDOFF','blended'),('WALK','targets'),('BRAKE','targets')]:
            needle=f'                log_debug_policy("{phase}",'
            assert changed.count(needle)==1
            changed=changed.replace(needle,'#if defined(AS2W_POLICY_HIMLOCO)\n'+f'                controller.accept_applied({variable});\n'+'#endif\n'+needle)
    assert original!=changed
    target=out/rel; target.parent.mkdir(parents=True,exist_ok=True);target.write_text(changed,encoding='utf-8')
    (out/(Path(rel).name+'.patch')).write_text(''.join(difflib.unified_diff(original.splitlines(True),changed.splitlines(True),fromfile=rel,tofile=rel)),encoding='utf-8')
    provenance[rel]={'source_sha256':hashlib.sha256(original.encode()).hexdigest(),'candidate_sha256':hashlib.sha256(changed.encode()).hexdigest()}
(out/'provenance.json').write_text(json.dumps(provenance,indent=2))
print('Isolated candidate generated. Compile/closed-loop validation NOT RUN.')
