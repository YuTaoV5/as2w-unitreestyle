"""Read-only rollout diagnostics. The PPO update uses the stock implementation."""
import hashlib
import json

import torch

from policy import gate

COMMANDS=("stand","forward","reverse","left","right","yaw_left","yaw_right","mixed")


def install_diagnostics(env, runner, out, args):
    alg=runner.alg
    update=alg.update
    completed=0
    digest=hashlib.sha256()
    for model in (alg.actor,alg.critic):
        for tensor in model.state_dict().values():
            digest.update(tensor.detach().cpu().contiguous().numpy().tobytes())
    (out/"initial.sha256").write_text(digest.hexdigest()+"\n")

    def measured_update(*positional,**keywords):
        nonlocal completed
        storage=alg.storage
        count=storage.step
        losses=update(*positional,**keywords)
        with torch.inference_mode():
            totals=torch.zeros(2,3,device=env.device)
            for start in range(0,count,4):
                stop=min(start+4,count)
                obs=storage.observations[start:stop].flatten(0,1)
                actions=storage.actions[start:stop].flatten(0,1)
                old_mean,old_std=[v[start:stop].flatten(0,1) for v in storage.distribution_params]
                new_mean=alg.actor(obs)
                kl=.5*((new_mean-old_mean)/old_std).square().sum(1)
                log_ratio=.5*(((actions-old_mean)/old_std).square()-((actions-new_mean)/old_std).square()).sum(1)
                outside=(log_ratio.exp()-1.).abs()>alg.clip_param
                active=gate(obs["policy"][:,:3]).view(-1)>0
                for row,mask in enumerate((torch.ones_like(active),active)):
                    totals[row]+=torch.stack((mask.sum(),(kl*mask).sum(),(outside*mask).sum()))
            sums=env.metrics.cpu().tolist()
            env.metrics.zero_()
        completed+=1
        result={"update":completed,"global_update":args.offset+completed}
        for name,(n,k,c) in zip(("all","residual_active"),totals.tolist()):
            result[name]={"post_update_kl":k/max(n,1),"post_update_clip_fraction":c/max(n,1),"samples":int(n)}
        result["commands"]={}
        for i,row in enumerate(sums):
            if row[0]:
                label=("flat/" if i<8 else "nonflat/")+COMMANDS[i%8]
                result["commands"][label]=dict(zip(env.metrics_names,[int(row[0])]+[v/row[0] for v in row[1:]]))
        with (out/"diagnostics.jsonl").open("a") as f:
            f.write(json.dumps(result,allow_nan=False)+"\n")
        if completed in (25,50):
            runner.save(str(out/f"update-{completed:02d}.pt"))
        return losses

    alg.update=measured_update
