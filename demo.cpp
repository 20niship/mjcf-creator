#include "mjcf/mjcf.hpp"
#include <fstream>
#include <iostream>

/**
 * @brief MJCF C++ライブラリの使用方法を示すデモプログラム
 *
 * このプログラムはPythonバージョンのgen_empty.pyの例を再現し、
 * C++での同等の機能をデモンストレーションします。
 */
int main() {
  std::cout << "MJCF C++ライブラリ デモ" << std::endl;
  std::cout << "=====================" << std::endl;

  // メインモデルを作成
  auto mujoco = std::make_shared<mjcf::Mujoco>("empty");

  auto asset     = mujoco->asset_;
  auto worldbody = mujoco->worldbody_;

  // レベル3 - アセット
  std::cout << "テクスチャとマテリアルを作成中..." << std::endl;

  // Using the new convenience constructor for procedural textures
  auto tex1 = std::make_shared<mjcf::Texture>("", mjcf::TextureBuiltin::Gradient, mjcf::TextureType::Skybox,
                                               std::array<double, 3>{1.0, 1.0, 1.0}, 
                                               std::array<double, 3>{0.0, 0.0, 0.0}, 100, 100);

  auto tex2 = std::make_shared<mjcf::Texture>("texgeom", mjcf::TextureBuiltin::Flat, mjcf::TextureType::Cube,
                                               std::array<double, 3>{0.8, 0.6, 0.4}, 
                                               std::array<double, 3>{0.8, 0.6, 0.4}, 127, 1278);
  tex2->mark    = "cross";
  tex2->markrgb = {1.0, 1.0, 1.0};
  tex2->random  = 0.01;

  // Using the static factory method for checkerboard texture
  auto tex3 = std::make_shared<mjcf::Texture>(
      mjcf::Texture::CheckerTexture("texplane", 
                                    std::array<double, 3>{0.0, 0.0, 0.0}, 
                                    std::array<double, 3>{0.8, 0.8, 0.8}, 100, 100));

  // Using the new convenience constructor for textured materials
  auto mat1 = std::make_shared<mjcf::Material>("MatPlane", "texplane", std::array<double, 2>{60.0, 60.0});
  mat1->reflectance = 0.5;
  mat1->shininess   = 1.0;
  mat1->specular    = 1.0;

  auto mat2 = std::make_shared<mjcf::Material>("geom", "texgeom");
  mat2->texuniform = true;

  asset->add_children({tex1, tex2, tex3, mat1, mat2});

  // ワールドボディ要素
  std::cout << "ワールドオブジェクトを作成中..." << std::endl;

  // Using the new convenience constructor for Light
  auto light = std::make_shared<mjcf::Light>(true, std::array<double, 3>{0.0, 0.0, 1.3}, 
                                              std::array<double, 3>{0.0, 0.0, -1.3}, 
                                              std::array<double, 3>{1.0, 1.0, 1.0});
  light->cutoff   = 100.0;
  light->exponent = 1.0;
  light->specular = {0.1, 0.1, 0.1};

  auto floor_geom         = std::make_shared<mjcf::Geom>();
  floor_geom->conaffinity = 1;
  floor_geom->condim      = 3;
  floor_geom->material    = "MatPlane";
  floor_geom->name        = "floor";
  floor_geom->pos         = {0.0, 0.0, 0.0};
  floor_geom->rgba        = {0.8, 0.9, 0.8, 1.0};
  floor_geom->size        = {40.0, 40.0, 40.0};
  floor_geom->type        = mjcf::GeomType::Plane;

  worldbody->add_children({light, floor_geom});

  // XMLを生成
  std::cout << "XMLを生成中..." << std::endl;
  std::string model_xml = mujoco->get_xml_text();

  // ファイルに保存
  std::ofstream outfile("empty-demo.xml");
  if(outfile.is_open()) {
    outfile << model_xml;
    outfile.close();
    std::cout << "MJCFモデルを 'empty-demo.xml' に保存しました" << std::endl;
  } else {
    std::cerr << "エラー: 書き込み用ファイルを開けませんでした" << std::endl;
    return 1;
  }

  // 情報を表示
  std::cout << "生成されたXMLの長さ: " << model_xml.length() << " 文字" << std::endl;
  std::cout << std::endl;

  // XMLのプレビューを表示
  std::cout << "XMLプレビュー:" << std::endl;
  std::cout << "============" << std::endl;
  std::cout << model_xml << std::endl;

  std::cout << std::endl;
  std::cout << "デモが正常に完了しました!" << std::endl;
  std::cout << "'empty-demo.xml' をMuJoCoで使用できます。" << std::endl;

  return 0;
}
