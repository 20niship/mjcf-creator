#include "mjcf/mjcf.hpp"
#include <iostream>

/**
 * @brief デフォルトコンストラクタと便利なコンストラクタのデモ
 * 
 * このプログラムは、新しい便利なコンストラクタとファクトリーメソッドの使い方を示します。
 */
int main() {
  std::cout << "便利なコンストラクタのデモ" << std::endl;
  std::cout << "========================" << std::endl;

  // 以前の方法（冗長）
  std::cout << "\n[以前の方法]" << std::endl;
  auto light_old = std::make_shared<mjcf::Light>();
  light_old->directional = true;
  light_old->pos = {0.0, 0.0, 2.0};
  light_old->dir = {0.0, 0.0, -1.0};
  light_old->diffuse = {1.0, 1.0, 1.0};

  // 新しい方法（簡潔）
  std::cout << "[新しい方法]" << std::endl;
  auto light_new = std::make_shared<mjcf::Light>(
      true,                              // directional
      std::array<double, 3>{0.0, 0.0, 2.0},  // pos
      std::array<double, 3>{0.0, 0.0, -1.0}, // dir
      std::array<double, 3>{1.0, 1.0, 1.0}   // diffuse
  );

  std::cout << "Lightの作成: " << light_new->get_xml_text() << std::endl;

  // Textureの便利なコンストラクタ
  std::cout << "\n[Texture便利コンストラクタ]" << std::endl;
  
  // ファイルベースのテクスチャ
  auto tex_file = std::make_shared<mjcf::Texture>("wood", "textures/wood.png");
  std::cout << "ファイルテクスチャ: " << tex_file->get_xml_text() << std::endl;

  // プロシージャルテクスチャ
  auto tex_gradient = std::make_shared<mjcf::Texture>(
      "sky", mjcf::TextureBuiltin::Gradient, mjcf::TextureType::Skybox,
      std::array<double, 3>{0.3, 0.5, 0.7},  // rgb1 (空の色)
      std::array<double, 3>{0.1, 0.1, 0.2},  // rgb2 (地平線の色)
      256, 256
  );
  std::cout << "グラデーションテクスチャ: " << tex_gradient->get_xml_text() << std::endl;

  // チェッカーボードテクスチャ（静的ファクトリーメソッド）
  std::cout << "\n[Texture静的ファクトリーメソッド]" << std::endl;
  auto tex_checker = std::make_shared<mjcf::Texture>(
      mjcf::Texture::CheckerTexture(
          "ground",
          std::array<double, 3>{0.2, 0.2, 0.2},  // 暗い色
          std::array<double, 3>{0.8, 0.8, 0.8},  // 明るい色
          512, 512
      )
  );
  std::cout << "チェッカーボード: " << tex_checker->get_xml_text() << std::endl;

  // Materialの便利なコンストラクタ
  std::cout << "\n[Material便利コンストラクタ]" << std::endl;
  
  // RGBAベースのマテリアル
  auto mat_color = std::make_shared<mjcf::Material>(
      "red_plastic",
      std::array<double, 4>{0.8, 0.1, 0.1, 1.0}
  );
  std::cout << "色マテリアル: " << mat_color->get_xml_text() << std::endl;

  // テクスチャ付きマテリアル
  auto mat_textured = std::make_shared<mjcf::Material>(
      "floor_mat",
      "ground",
      std::array<double, 2>{20.0, 20.0}
  );
  std::cout << "テクスチャマテリアル: " << mat_textured->get_xml_text() << std::endl;

  std::cout << "\nデモ完了!" << std::endl;
  std::cout << "\n便利なコンストラクタにより、コードが簡潔で読みやすくなりました。" << std::endl;

  return 0;
}
