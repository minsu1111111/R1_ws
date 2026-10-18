"""LeRobot 데이터셋 기록 (lerobot 0.4.1, 데이터셋 형식 v3.0).

    rec = EpisodeRecorder(root, fps=30, state_names=[...], action_names=[...], image_shapes={"head": (240, 320, 3)},
                          task="push the red box to the green circle")
    rec.add(state, action, {"head": rgb})   # 매 프레임
    rec.save()  /  rec.discard()            # 에피소드 끝
    rec.close()                             # 프로그램 끝 (finalize 필수: parquet footer 기록)

- root 에 데이터셋이 이미 있으면 이어서 기록한다 (특징 정의가 같아야 함).
- 에피소드별 부가 정보(상자 시작·끝 위치, 성공 여부)는 root 옆 <이름>.episodes.jsonl 에 남긴다.
"""

import json
from pathlib import Path

import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset


class EpisodeRecorder:
    def __init__(
        self,
        root: str | Path,
        fps: int,
        state_names: list[str],
        action_names: list[str],
        image_shapes: dict[str, tuple[int, int, int]],
        task: str,
        robot_type: str = "unitree_r1",
    ):
        self.root = Path(root)
        self.task = task
        self.log_path = self.root.parent / f"{self.root.name}.episodes.jsonl"
        features = {
            "observation.state": {"dtype": "float32", "shape": (len(state_names),), "names": state_names},
            "action": {"dtype": "float32", "shape": (len(action_names),), "names": action_names},
        }
        for name, shape in image_shapes.items():
            features[f"observation.images.{name}"] = {
                "dtype": "video", "shape": tuple(shape), "names": ["height", "width", "channels"],
            }
        repo_id = f"local/{self.root.name}"

        if (self.root / "meta" / "info.json").exists():
            self.ds = LeRobotDataset(repo_id, root=self.root)
            self.ds.start_image_writer(num_threads=4)
            for k, v in features.items():
                have = self.ds.features.get(k)
                if have is None or tuple(have["shape"]) != tuple(v["shape"]):
                    raise ValueError(f"기존 데이터셋과 특징이 다름: {k} {have} vs {v}. 다른 --name 을 쓰세요.")
            print(f"[rec] resume {self.root} ({self.ds.num_episodes} episodes)")
        else:
            self.root.parent.mkdir(parents=True, exist_ok=True)
            self.ds = LeRobotDataset.create(
                repo_id, fps=fps, features=features, root=self.root, robot_type=robot_type,
                use_videos=True, image_writer_threads=4,
            )
            print(f"[rec] new dataset {self.root}")
        self.frames = 0

    @property
    def num_episodes(self) -> int:
        return self.ds.num_episodes

    def add(self, state: np.ndarray, action: np.ndarray, images: dict[str, np.ndarray]):
        frame = {
            "observation.state": np.asarray(state, dtype=np.float32),
            "action": np.asarray(action, dtype=np.float32),
            "task": self.task,
        }
        for name, img in images.items():
            frame[f"observation.images.{name}"] = img
        self.ds.add_frame(frame)
        self.frames += 1

    def save(self, info: dict | None = None):
        n = self.frames
        self.ds.save_episode()
        with open(self.log_path, "a") as f:
            f.write(json.dumps({"episode_index": self.ds.num_episodes - 1, "frames": n, **(info or {})}) + "\n")
        self.frames = 0

    def discard(self):
        self.ds.clear_episode_buffer()
        self.frames = 0

    def close(self):
        if self.frames:
            self.discard()
        self.ds.stop_image_writer()
        self.ds.finalize()
