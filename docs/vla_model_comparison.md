# VLA 모델 비교: OpenVLA-OFT vs openpi (π₀)

M0609 + RG2 + RealSense D435 × 2 환경 기준 (RTX 4090 24GB, PyTorch 친숙)

---

## 아키텍처 비교

| 항목 | OpenVLA-OFT | openpi (π₀) |
|------|-------------|-------------|
| 기반 아키텍처 | LLM 기반 (Llama 2 7B + DINOv2 + SigLIP) | Flow Matching VLA |
| 파라미터 규모 | ~7B | ~7B (π₀ 기준) |
| 액션 예측 방식 | Autoregressive (토큰 디코딩) | Diffusion / Flow matching |
| 언어 조건부 | ✅ 자연어 명령 | ✅ 자연어 명령 |
| 레퍼런스 | [moojink/openvla-oft](https://github.com/moojink/openvla-oft) | [Physical-Intelligence/openpi](https://github.com/Physical-Intelligence/openpi) |

---

## 멀티 카메라 지원

| 항목 | OpenVLA-OFT | openpi (π₀) |
|------|-------------|-------------|
| 멀티 카메라 | ✅ (최대 3대) | ✅ (DROID/ALOHA 예제) |
| wrist + global 구성 | ✅ LIBERO 예제 기준 | ✅ ALOHA 예제 기준 |
| 입력 해상도 | 224 × 224 | 224 × 224 (일반적) |
| depth 이미지 | ❌ RGB only | ❌ RGB only |

---

## 학습 환경 (RTX 4090 기준)

| 항목 | OpenVLA-OFT | openpi (π₀) |
|------|-------------|-------------|
| LoRA fine-tuning VRAM | ~27GB (BF16) → **4-bit 양자화 시 ~16GB** | **공식 22.5GB** (LoRA) |
| Full fine-tuning VRAM | ~80GB+ | ~70GB+ |
| RTX 4090 (24GB) 가능 여부 | ⚠️ 4-bit 양자화 필요 (비공식) | ✅ 공식 LoRA 지원 |
| 단일 GPU 학습 | 가능 (양자화 필요) | 공식 지원 |
| 양자화 지원 | `load_in_4bit` / `load_in_8bit` | QLoRA 미언급 |

---

## 개발 환경 및 생태계

| 항목 | OpenVLA-OFT | openpi (π₀) |
|------|-------------|-------------|
| 프레임워크 | PyTorch ✅ | **JAX** (권장) / PyTorch (제한적) |
| Mixed-precision 학습 | PyTorch 지원 | JAX만 지원, PyTorch 미지원 |
| HuggingFace 연동 | ✅ (Transformers, PEFT, bitsandbytes) | △ |
| 커뮤니티 | 활성화 | 비교적 신규 |

---

## 우리 셋업과의 적합성

| 항목 | OpenVLA-OFT | openpi (π₀) |
|------|-------------|-------------|
| M0609 + RG2 joint 호환 | 커스텀 클래스 필요 | 커스텀 클래스 필요 |
| camera_gripper + camera_global | ✅ | ✅ |
| /dsr01/joint_states 연동 | ✅ | ✅ |
| PyTorch 친숙도 활용 | ✅ 직접 활용 | ❌ JAX 신규 학습 필요 |

---

## 결론

| | OpenVLA-OFT | openpi (π₀) |
|---|---|---|
| **RTX 4090 실현 가능성** | ⚠️ 4-bit 양자화 우회 필요 | ✅ 공식 지원 |
| **PyTorch 개발 편의성** | ✅ | ❌ (JAX 권장) |
| **종합 추천** | PyTorch 익숙 + HF 생태계 활용 시 | JAX 학습 감수 가능 시 |

> **현재 선택**: 미확정 — 데이터 수집 파이프라인 완성 후 소규모 테스트로 결정 예정
