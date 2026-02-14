#!/usr/bin/env python3
"""
Script to download and convert MiDaS model to ONNX format.

MiDaS: A PyTorch implementation of the monocular depth estimation model.
Model: https://github.com/isl-org/MiDaS

This script downloads a pretrained MiDaS model and converts it to ONNX format
for deployment with ONNX Runtime.

Available models:
- "midas_v3_small": Smallest model, fastest inference (~30ms on GPU)
- "dpt_hybrid": Medium model, better accuracy
- "dpt_large": Largest model, best accuracy but slowest
"""

import os
import sys
import argparse
import torch
import numpy as np
from pathlib import Path

# Try importing required libraries
try:
    import timm
    import cv2
    import onnx
    import onnxruntime as ort
except ImportError as e:
    print(f"Error: Missing required package: {e}")
    print("Install with: pip install timm opencv-python onnx onnxruntime")
    sys.exit(1)


class MiDaSDownloader:
    """Download and convert MiDaS models to ONNX format."""
    
    # Model URLs from official MiDaS repository
    MODELS = {
        "midas_v3_small": {
            "url": "https://github.com/isl-org/MiDaS/releases/download/v3_1/midas_v31_small_256.pt",
            "input_size": (256, 256),
            "type": "small"
        },
        "dpt_hybrid": {
            "url": "https://github.com/isl-org/MiDaS/releases/download/v3/dpt_hybrid-midas-501f0c75.pt",
            "input_size": (384, 384),
            "type": "hybrid"
        },
        "dpt_large": {
            "url": "https://github.com/isl-org/MiDaS/releases/download/v3/dpt_large-midas-2f21e75d.pt",
            "input_size": (384, 384),
            "type": "large"
        }
    }
    
    def __init__(self, model_dir: str = "models"):
        self.model_dir = Path(model_dir)
        self.model_dir.mkdir(parents=True, exist_ok=True)
        
    def download_model(self, model_name: str) -> str:
        """Download MiDaS model from official repository."""
        if model_name not in self.MODELS:
            raise ValueError(f"Unknown model: {model_name}. Available: {list(self.MODELS.keys())}")
        
        model_info = self.MODELS[model_name]
        output_path = self.model_dir / f"{model_name}.pt"
        
        if output_path.exists():
            print(f"✓ Model already exists: {output_path}")
            return str(output_path)
        
        print(f"📥 Downloading {model_name} from official repository...")
        print(f"   URL: {model_info['url']}")
        
        try:
            import urllib.request
            urllib.request.urlretrieve(model_info['url'], str(output_path))
            print(f"✓ Downloaded to: {output_path}")
            return str(output_path)
        except Exception as e:
            print(f"✗ Failed to download: {e}")
            print("  Try manual download and place in models/ directory")
            return None
        
    def convert_to_onnx(self, model_name: str, pt_path: str) -> str:
        """Convert PyTorch MiDaS model to ONNX format."""
        if model_name not in self.MODELS:
            raise ValueError(f"Unknown model: {model_name}")
        
        model_info = self.MODELS[model_name]
        output_path = self.model_dir / f"{model_name}.onnx"
        
        if output_path.exists():
            print(f"✓ ONNX model already exists: {output_path}")
            return str(output_path)
        
        print(f"\n🔄 Converting {model_name} to ONNX format...")
        print(f"   Input model: {pt_path}")
        print(f"   Output model: {output_path}")
        
        try:
            # Load model
            print("  Loading PyTorch model...")
            device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
            print(f"  Using device: {device}")
            
            # Import MiDaS model loader
            import torch
            from torchvision import transforms
            
            # Load model based on type
            if model_info["type"] == "small":
                model = self._load_midas_small(pt_path, device)
            elif model_info["type"] == "hybrid":
                model = self._load_midas_hybrid(pt_path, device)
            elif model_info["type"] == "large":
                model = self._load_midas_large(pt_path, device)
            
            model.eval()
            
            # Create dummy input
            h, w = model_info["input_size"]
            dummy_input = torch.randn(1, 3, h, w).to(device)
            
            # Export to ONNX
            print(f"  Exporting to ONNX (input shape: 1x3x{h}x{w})...")
            torch.onnx.export(
                model,
                dummy_input,
                str(output_path),
                input_names=['image'],
                output_names=['depth'],
                dynamic_axes={'image': {0: 'batch_size'}, 'depth': {0: 'batch_size'}},
                verbose=False,
                opset_version=12
            )
            
            print(f"✓ ONNX conversion successful!")
            print(f"✓ Model saved to: {output_path}")
            
            # Verify ONNX model
            print(f"  Verifying ONNX model...")
            onnx_model = onnx.load(str(output_path))
            onnx.checker.check_model(onnx_model)
            print(f"✓ ONNX model verification passed!")
            
            # Test ONNX Runtime
            print(f"  Testing with ONNX Runtime...")
            session = ort.InferenceSession(str(output_path))
            input_name = session.get_inputs()[0].name
            test_input = np.random.randn(1, 3, h, w).astype(np.float32)
            outputs = session.run(None, {input_name: test_input})
            print(f"✓ ONNX Runtime test passed! Output shape: {np.array(outputs[0]).shape}")
            
            return str(output_path)
            
        except Exception as e:
            print(f"✗ Conversion failed: {e}")
            import traceback
            traceback.print_exc()
            return None
    
    @staticmethod
    def _load_midas_small(model_path: str, device):
        """Load MiDaS Small model."""
        import torch
        model = torch.hub.load("intel-isl/MiDaS", "midas_v31_small")
        model.to(device)
        return model
    
    @staticmethod
    def _load_midas_hybrid(model_path: str, device):
        """Load MiDaS Hybrid model."""
        import torch
        model = torch.hub.load("intel-isl/MiDaS", "dpt_hybrid")
        model.to(device)
        return model
    
    @staticmethod
    def _load_midas_large(model_path: str, device):
        """Load MiDaS Large model."""
        import torch
        model = torch.hub.load("intel-isl/MiDaS", "dpt_large")
        model.to(device)
        return model


def main():
    parser = argparse.ArgumentParser(
        description="Download and convert MiDaS model to ONNX format"
    )
    parser.add_argument(
        '--model',
        type=str,
        default='midas_v3_small',
        choices=['midas_v3_small', 'dpt_hybrid', 'dpt_large'],
        help='Model to download and convert'
    )
    parser.add_argument(
        '--model-dir',
        type=str,
        default='models',
        help='Directory to save models'
    )
    parser.add_argument(
        '--skip-download',
        action='store_true',
        help='Skip downloading, assume model exists in model_dir'
    )
    
    args = parser.parse_args()
    
    print("=" * 70)
    print("MiDaS Model Downloader and ONNX Converter")
    print("=" * 70)
    
    downloader = MiDaSDownloader(args.model_dir)
    
    # Step 1: Download
    if not args.skip_download:
        pt_path = downloader.download_model(args.model)
        if not pt_path:
            print("✗ Failed to download model")
            sys.exit(1)
    else:
        pt_path = downloader.model_dir / f"{args.model}.pt"
        if not pt_path.exists():
            print(f"✗ Model file not found: {pt_path}")
            sys.exit(1)
        print(f"✓ Using existing model: {pt_path}")
    
    # Step 2: Convert
    onnx_path = downloader.convert_to_onnx(args.model, str(pt_path))
    if not onnx_path:
        print("✗ Failed to convert model to ONNX")
        sys.exit(1)
    
    print("\n" + "=" * 70)
    print("✓ SUCCESS!")
    print("=" * 70)
    print(f"Model ready at: {onnx_path}")
    print(f"Model name: {args.model}")
    print(f"File size: {os.path.getsize(onnx_path) / 1e6:.1f} MB")
    print("\nUpdate config file to use this model:")
    print(f"  depth_estimator:")
    print(f"    model_name: {args.model}")
    print(f"    model_path: {onnx_path}")


if __name__ == '__main__':
    main()
