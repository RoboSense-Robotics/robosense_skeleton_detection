import os
import numpy as np
import argparse
import pickle
import tensorrt as trt

print(trt.__version__)



def check_fp32(name: str):
    fp32_names = [
        "/Reshape",
        "/mlp/mlp.0/Abs",
        "/mlp/mlp.0/Pow",
        "/mlp/mlp.0/ReduceSum",
        "/mlp/mlp.0/Pow_1",
        "/mlp/mlp.0/Mul",
        "/mlp/mlp.0/Clip",
        "/mlp/mlp.0/Div",
        "/mlp/mlp.0/Mul_1",
        "/mlp/mlp.1/MatMul",
        "/gau/ln/Abs",
        "/gau/ln/Pow",
        "/gau/ln/ReduceSum",
        "/gau/ln/Pow_1",
        "/gau/ln/Mul",
        "/gau/ln/Clip",
        "/gau/ln/Div",
        "/gau/ln/Mul_1",
        "/gau/uv/MatMul",
        "/gau/act_fn/Sigmoid",
        "/gau/act_fn/Mul",
        "/gau/Split",
        "/gau/Unsqueeze",
        "/gau/Mul",
        "/gau/Add",
        "/gau/Split_1",
        "/gau/Squeeze_1",
        "/gau/Transpose",
        "/gau/Squeeze",
        "/gau/MatMul",
        "/gau/Div",
        "/gau/Relu",
        "/gau/Mul_1",
        "/gau/MatMul_1",
        "/gau/Mul_2",
        "/gau/o/MatMul",
        "/gau/res_scale/Mul",
        "/gau/Add_1",
        "/cls_y/MatMul",
        "/cls_x/MatMul"
    ]
    for i in fp32_names:
        if name == i:
            return True
    return False


def ONNX2TRT(onnx_file, trt_file):
    ''' convert onnx to tensorrt engine, use mode of ['fp32', 'fp16', 'int8']
    :return: trt engine
    '''

    # G_LOGGER = trt.Logger(trt.Logger.VERBOSE)
    G_LOGGER = trt.Logger(trt.Logger.INFO)
    # TRT7中的onnx解析器的network，需要指定EXPLICIT_BATCH
    EXPLICIT_BATCH = 1 << (int)(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
    with trt.Builder(G_LOGGER) as builder, builder.create_network(EXPLICIT_BATCH) as network, \
            trt.OnnxParser(network, G_LOGGER) as parser:

        config = builder.create_builder_config()
        config.profiling_verbosity = trt.ProfilingVerbosity.DETAILED  # 为后续节点作图可视化必要配置
        config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, (2 ** 32))
        config.default_device_type = trt.DeviceType.GPU
        config.clear_flag(trt.BuilderFlag.FP16)
        config.clear_flag(trt.BuilderFlag.TF32)

        assert (builder.platform_has_fast_fp16), "not support fp16"
        config.set_flag(trt.BuilderFlag.FP16)

        print('Loading ONNX file from path {}...'.format(onnx_file))
        with open(onnx_file, 'rb') as model:
            print('Beginning ONNX file parsing')
            if not parser.parse(model.read()):
                for e in range(parser.num_errors):
                    print(parser.get_error(e))
                raise TypeError("Parser parse failed.")

        config.set_flag(trt.BuilderFlag.PREFER_PRECISION_CONSTRAINTS)
        for i in range(network.num_layers):
            layer = network[i]
            name = layer.name
            if check_fp32(name):
                print(f"set layer {name} to fp32")
                layer.precision = trt.float32

        print('Completed parsing of ONNX file')

        engine_data = builder.build_serialized_network(network, config)
        with open(trt_file, "wb") as f:
            f.write(engine_data)
        print('Completed convert of engine file')


def convert_int8():
    parser = argparse.ArgumentParser(description="Pytorch2TensorRT args")
    parser.add_argument("--onnx_file", type=str, default='', help='onnx_file_path')
    parser.add_argument("--engine_file", type=str, default='', help='engine_file_path')

    args = parser.parse_args()
    ONNX2TRT(onnx_file=args.onnx_file, trt_file=args.engine_file)


if __name__ == "__main__":
    convert_int8()

    
