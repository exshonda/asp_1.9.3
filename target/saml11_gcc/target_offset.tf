$
$       オフセットファイル生成用テンプレートファイル（SAML11用）
$

$
$  標準テンプレートファイルのインクルード
$
$INCLUDE "kernel/genoffset.tf"$

$ 
$  コア依存テンプレートのインクルード（ARM-M用）
$ 
$INCLUDE"../../arch/arm_m_gcc/common/core_offset.tf"$
