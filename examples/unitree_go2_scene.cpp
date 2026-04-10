/**
 * Unitree Go2 シーン生成サンプル
 *
 * add_xml_file() を使ってGo2のMJCFモデルをシーンに取り込み、
 * 床プリミティブを追加した統合XMLを生成する。
 */

#include "mjcf/mjcf.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

int main(int argc, char* argv[]) {
  // Go2 XMLファイルのパス（引数で上書き可能）
  std::string go2_xml_path = "assets/unitree_go2/go2.xml";
  if(argc > 1) go2_xml_path = argv[1];

  if(!std::filesystem::exists(go2_xml_path)) {
    std::cerr << "ERROR: Go2 XML not found: " << go2_xml_path << std::endl;
    std::cerr << "Run from the mjcf-creator root directory." << std::endl;
    return 1;
  }

  // ===== メインシーン作成 =====
  auto scene = std::make_shared<mjcf::Mujoco>("go2_scene");

  // シミュレーション設定
  scene->option_->timestep   = 0.002;
  scene->option_->integrator = mjcf::IntegratorType::RK4;

  // ----- 床テクスチャ & マテリアル -----
  auto floor_tex = std::make_shared<mjcf::Texture>();
  floor_tex->name    = "floor_tex";
  floor_tex->builtin = mjcf::TextureBuiltin::Checker;
  floor_tex->type    = mjcf::TextureType::TwoD;
  floor_tex->width   = 512;
  floor_tex->height  = 512;
  floor_tex->rgb1    = {0.2, 0.3, 0.4};
  floor_tex->rgb2    = {0.1, 0.15, 0.2};
  scene->add_asset(floor_tex);

  auto floor_mat = std::make_shared<mjcf::Material>();
  floor_mat->name    = "floor_mat";
  floor_mat->texture = "floor_tex";
  scene->add_asset(floor_mat);

  // ----- 床ジオム（プリミティブ）-----
  auto floor_geom   = mjcf::Geom::Plane("floor", {20.0, 20.0, 0.1}, {0, 0, 0}, {0.5, 0.5, 0.5, 1.0});
  floor_geom->material = "floor_mat";
  scene->worldbody_->add_child(floor_geom);

  // ----- ライト -----
  auto sun = std::make_shared<mjcf::Light>();
  sun->name      = "sun";
  sun->pos       = {0, 0, 5};
  sun->dir       = {0, 0, -1};
  sun->diffuse   = {0.8, 0.8, 0.8};
  sun->directional = true;
  scene->worldbody_->add_child(sun);

  // ===== Unitree Go2 を取り込む =====
  std::cout << "Loading Go2 from: " << go2_xml_path << std::endl;
  scene->add_xml_file(go2_xml_path, nullptr, "");  // プレフィックスなし（Go2側のnameをそのまま使用）

  // ===== 出力 =====
  const std::string out_path = "go2_scene.xml";
  const std::string xml_text = scene->get_xml_text();

  std::ofstream out(out_path);
  out << xml_text;
  out.close();

  std::cout << "Generated: " << out_path << " (" << xml_text.size() << " bytes)" << std::endl;

  // ===== 簡易検証 =====
  bool ok = true;
  auto check = [&](const std::string& needle, const std::string& desc) {
    if(xml_text.find(needle) == std::string::npos) {
      std::cerr << "  [FAIL] missing: " << desc << " (\"" << needle << "\")" << std::endl;
      ok = false;
    } else {
      std::cout << "  [OK]   " << desc << std::endl;
    }
  };

  std::cout << "\n--- 検証 ---" << std::endl;
  check("floor",           "床ジオム");
  check("floor_tex",       "床テクスチャ");
  check("floor_mat",       "床マテリアル");
  check("base_0.obj",      "Go2メッシュ (base_0.obj) が絶対パスで存在");
  check("go2",             "Go2ボディ");
  check("FR_hip",          "前右股関節 (FR_hip)");
  check("FL_hip",          "前左股関節モーター (FL_hip)");
  check("<actuator>",      "アクチュエータセクション");
  check("<default",        "デフォルトセクション");

  // メッシュパスが絶対パスになっているか
  if(xml_text.find("file=\"base_0.obj\"") != std::string::npos) {
    std::cerr << "  [FAIL] mesh path is still relative (base_0.obj)" << std::endl;
    ok = false;
  } else {
    std::cout << "  [OK]   メッシュパスが絶対パスに解決済み" << std::endl;
  }

  std::cout << "\n" << (ok ? "[PASS] すべての検証が通りました" : "[FAIL] 一部の検証が失敗しました") << std::endl;
  return ok ? 0 : 1;
}
