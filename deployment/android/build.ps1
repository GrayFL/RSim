param([Parameter(Mandatory=$true)][string]$SdkRoot,
      [string]$JavaRoot='D:\LAB\Java\jdk-21',
      [string]$Output=(Join-Path $PSScriptRoot '..\..\assets\android'))
$ErrorActionPreference='Stop'
$tools=Join-Path $SdkRoot 'android-15'
$platform=Join-Path $SdkRoot 'android-12\android.jar'
New-Item -ItemType Directory -Force -Path $Output | Out-Null
$Output=(Resolve-Path $Output).Path
$classes=Join-Path $Output 'classes'
New-Item -ItemType Directory -Force -Path $classes | Out-Null
& "$tools\aapt2.exe" compile --dir "$PSScriptRoot\res" -o "$Output\resources.zip"
if ($LASTEXITCODE) { throw 'Resource compilation failed' }
& "$tools\aapt2.exe" link -o "$Output\unsigned.apk" --manifest "$PSScriptRoot\AndroidManifest.xml" -I $platform "$Output\resources.zip"
if ($LASTEXITCODE) { throw 'Resource linking failed' }
& "$JavaRoot\bin\javac.exe" -encoding UTF-8 -source 8 -target 8 -classpath $platform -d $classes "$PSScriptRoot\src\org\rsim\console\MainActivity.java"
if ($LASTEXITCODE) { throw 'Java compilation failed' }
& "$JavaRoot\bin\java.exe" -cp "$tools\lib\d8.jar" com.android.tools.r8.D8 --min-api 26 --lib $platform --output $Output "$classes\org\rsim\console\MainActivity.class"
if ($LASTEXITCODE) { throw 'DEX compilation failed' }
& "$JavaRoot\bin\jar.exe" uf "$Output\unsigned.apk" -C $Output classes.dex
& "$tools\zipalign.exe" -f 4 "$Output\unsigned.apk" "$Output\aligned.apk"
$keystore=Join-Path $Output 'rsim-local.keystore'
if (!(Test-Path $keystore)) {
    & "$JavaRoot\bin\keytool.exe" -genkeypair -keystore $keystore -storepass android -keypass android -alias rsim-local -dname 'CN=RSim local robot launcher' -keyalg RSA -keysize 2048 -validity 10000
}
& "$JavaRoot\bin\java.exe" -jar "$tools\lib\apksigner.jar" sign --ks $keystore --ks-pass pass:android --key-pass pass:android --out "$Output\rsim-console.apk" "$Output\aligned.apk"
if ($LASTEXITCODE) { throw 'APK signing failed' }
& "$JavaRoot\bin\java.exe" -jar "$tools\lib\apksigner.jar" verify --verbose "$Output\rsim-console.apk"
if ($LASTEXITCODE) { throw 'APK verification failed' }
Get-FileHash "$Output\rsim-console.apk" -Algorithm SHA256
