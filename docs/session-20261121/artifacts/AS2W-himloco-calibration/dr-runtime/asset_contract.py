from pathlib import Path

MU025_SHA = "d48020cce1cbe59655c3bd95800f73b11b47026664deb941dfd0ebf6a1858034"

def configure_asset(cfg, asset, usd_dir):
    from isaaclab.sim import UsdFileCfg
    import hashlib
    asset = Path(asset).resolve(strict=True)
    if asset.suffix.lower() == ".urdf":
        cfg.scene.robot.spawn.asset_path = str(asset)
        cfg.scene.robot.spawn.usd_dir = str(usd_dir)
    elif asset.suffix.lower() in (".usd", ".usda", ".usdc"):
        if hashlib.sha256(asset.read_bytes()).hexdigest() != MU025_SHA:
            raise ValueError("Unsupported USD contact contract")
        cfg.scene.robot.spawn = UsdFileCfg(usd_path=str(asset), activate_contact_sensors=True)
        material = cfg.scene.terrain.physics_material
        material.static_friction = material.dynamic_friction = .25
        material.friction_combine_mode = "min"
    else:
        raise ValueError(f"Unsupported asset: {asset}")

def apply_shared_plane_contract(asset):
    if Path(asset).suffix.lower() == ".urdf":
        return
    import omni.usd
    from pxr import PhysxSchema, UsdShade
    stage = omni.usd.get_context().get_stage()
    plane = stage.GetPrimAtPath("/World/ground/terrain/GroundPlane/CollisionPlane")
    material, _ = UsdShade.MaterialBindingAPI(plane).ComputeBoundMaterial("physics")
    api = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
    api.CreateFrictionCombineModeAttr().Set("min")
    assert api.GetFrictionCombineModeAttr().Get() == "min"
